# SPDX-License-Identifier: Apache-2.0
if(CONFIG_MAX30003)
  zephyr_include_directories(./include)

  zephyr_library()
  zephyr_library_sources(
    src/max30003.c
  )
endif()
