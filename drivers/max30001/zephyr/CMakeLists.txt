# SPDX-License-Identifier: Apache-2.0
if(CONFIG_MAX30001)
  zephyr_include_directories(./include)

  zephyr_library()
  zephyr_library_sources(
    src/max30001.c
  )
endif()
