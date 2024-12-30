# SPDX-License-Identifier: Apache-2.0
if(CONFIG_ADS1299)
zephyr_include_directories(./include)

zephyr_library()
zephyr_library_sources(
  src/ads1299.c
)
endif()

