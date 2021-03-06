/*
 * Copyright (c) 2018 Intel Corporation.
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <pm/pm.h>
#include <pm/policy.h>
#include <spinlock.h>
#include <sys_clock.h>
#include <sys/__assert.h>
#include <sys/time_units.h>
#include <sys/atomic.h>
#include <toolchain.h>

#define DT_SUB_LOCK_INIT(node_id)				\
	{ .state = PM_STATE_DT_INIT(node_id),			\
	  .substate_id = DT_PROP_OR(node_id, substate_id, 0),	\
	  .lock = ATOMIC_INIT(0),				\
	},

/**
 * State and substate lock structure.
 *
 * This struct is associating a reference counting to each <state,substate>
 * couple to be used with the pm_policy_substate_lock_* functions.
 *
 * Operations on this array are in the order of O(n) with the number of power
 * states and this is mostly due to the random nature of the substate value
 * (that can be anything from a small integer value to a bitmask). We can
 * probably do better with an hashmap.
 */
static struct {
	enum pm_state state;
	uint8_t substate_id;
	atomic_t lock;
} substate_lock_t[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_power_state, DT_SUB_LOCK_INIT)
};

/** Lock to synchronize access to the latency request list. */
static struct k_spinlock latency_lock;
/** List of maximum latency requests. */
static sys_slist_t latency_reqs;
/** Maximum CPU latency in ticks */
static int32_t max_latency_ticks = K_TICKS_FOREVER;
/** Callback to notify when maximum latency changes. */
static pm_policy_latency_changed_cb_t latency_changed_cb;

/** @brief Update maximum allowed latency. */
static void update_max_latency(void)
{
	int32_t new_max_latency_ticks = K_TICKS_FOREVER;
	struct pm_policy_latency_request *req;

	SYS_SLIST_FOR_EACH_CONTAINER(&latency_reqs, req, node) {
		if ((new_max_latency_ticks == K_TICKS_FOREVER) ||
		    ((int32_t)req->value < new_max_latency_ticks)) {
			new_max_latency_ticks = (int32_t)req->value;
		}
	}

	if ((latency_changed_cb != NULL) &&
	    (max_latency_ticks != new_max_latency_ticks)) {
		int32_t latency_us;

		if (new_max_latency_ticks == K_TICKS_FOREVER) {
			latency_us = SYS_FOREVER_US;
		} else {
			latency_us = (int32_t)k_ticks_to_us_ceil32(new_max_latency_ticks);
		}

		latency_changed_cb(latency_us);
	}

	max_latency_ticks = new_max_latency_ticks;
}

#ifdef CONFIG_PM_POLICY_DEFAULT
const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
	uint8_t num_cpu_states;
	const struct pm_state_info *cpu_states;

	num_cpu_states = pm_state_cpu_get_all(cpu, &cpu_states);

	for (int16_t i = (int16_t)num_cpu_states - 1; i >= 0; i--) {
		const struct pm_state_info *state = &cpu_states[i];
		uint32_t min_residency, exit_latency;

		/* check if there is a lock on state + substate */
		if (pm_policy_state_lock_is_active(state->state, state->substate_id)) {
			continue;
		}

		min_residency = k_us_to_ticks_ceil32(state->min_residency_us);
		exit_latency = k_us_to_ticks_ceil32(state->exit_latency_us);

		/* skip state if it brings too much latency */
		if ((max_latency_ticks != K_TICKS_FOREVER) &&
		    (exit_latency >= max_latency_ticks)) {
			continue;
		}

		if ((ticks == K_TICKS_FOREVER) ||
		    (ticks >= (min_residency + exit_latency))) {
			return state;
		}
	}

	return NULL;
}
#endif

void pm_policy_state_lock_get(enum pm_state state, uint8_t substate_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(substate_lock_t); i++) {
		if (substate_lock_t[i].state == state &&
		   (substate_lock_t[i].substate_id == substate_id ||
		    substate_id == PM_ALL_SUBSTATES)) {
			atomic_inc(&substate_lock_t[i].lock);
		}
	}
}

void pm_policy_state_lock_put(enum pm_state state, uint8_t substate_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(substate_lock_t); i++) {
		if (substate_lock_t[i].state == state &&
		   (substate_lock_t[i].substate_id == substate_id ||
		    substate_id == PM_ALL_SUBSTATES)) {
			atomic_t cnt = atomic_dec(&substate_lock_t[i].lock);

			ARG_UNUSED(cnt);

			__ASSERT(cnt >= 1, "Unbalanced state lock get/put");
		}
	}
}

bool pm_policy_state_lock_is_active(enum pm_state state, uint8_t substate_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(substate_lock_t); i++) {
		if (substate_lock_t[i].state == state &&
		   (substate_lock_t[i].substate_id == substate_id ||
		    substate_id == PM_ALL_SUBSTATES)) {
			return (atomic_get(&substate_lock_t[i].lock) != 0);
		}
	}

	return false;
}

void pm_policy_latency_request_add(struct pm_policy_latency_request *req,
				   uint32_t value)
{
	req->value = k_us_to_ticks_ceil32(value);

	k_spinlock_key_t key = k_spin_lock(&latency_lock);

	sys_slist_append(&latency_reqs, &req->node);
	update_max_latency();

	k_spin_unlock(&latency_lock, key);
}

void pm_policy_latency_request_update(struct pm_policy_latency_request *req,
				      uint32_t value)
{
	k_spinlock_key_t key = k_spin_lock(&latency_lock);

	req->value = k_us_to_ticks_ceil32(value);
	update_max_latency();

	k_spin_unlock(&latency_lock, key);
}

void pm_policy_latency_request_remove(struct pm_policy_latency_request *req)
{
	k_spinlock_key_t key = k_spin_lock(&latency_lock);

	(void)sys_slist_find_and_remove(&latency_reqs, &req->node);
	update_max_latency();

	k_spin_unlock(&latency_lock, key);
}

void pm_policy_latency_changed(pm_policy_latency_changed_cb_t cb)
{
	k_spinlock_key_t key = k_spin_lock(&latency_lock);

	latency_changed_cb = cb;

	k_spin_unlock(&latency_lock, key);
}
