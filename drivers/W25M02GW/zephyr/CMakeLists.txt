# SPDX-License-Identifier: Apache-2.0
if(CONFIG_W25M02GW)
zephyr_include_directories(./include)

zephyr_library()
zephyr_library_sources(
  src/w25m02gw.c
)
endif()