# Copyright (c) 2025 Sentry Technologies ApS
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(sch16t.c sch16t_bus.c sch16t_decoder.c sch16t_submit.c)

zephyr_library_sources_ifdef(CONFIG_SCH16T_STREAM sch16t_stream.c)
