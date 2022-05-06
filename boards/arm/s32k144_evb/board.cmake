# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=S32K14")
board_runner_args(pyocd "--target=s32k144evb")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

