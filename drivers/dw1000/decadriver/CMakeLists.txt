# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_DW1000 deca_device.c)
zephyr_library_sources_ifdef(CONFIG_DW1000 deca_params_init.c)
