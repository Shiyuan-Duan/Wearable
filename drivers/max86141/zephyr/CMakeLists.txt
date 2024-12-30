# SPDX-License-Identifier: Apache-2.0
if(CONFIG_MAX86141)
  zephyr_include_directories(./include)

  zephyr_library()
  zephyr_library_sources(
    src/max86141.c
  )
endif()

