# Copyright (c) 2025 Sentry Technologies
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories_ifdef(CONFIG_WITMOTION .)

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_HWT905TTL hwt905ttl)
# zephyr-keep-sorted-stop
