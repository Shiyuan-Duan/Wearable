# SPDX-License-Identifier: Apache-2.0
if(CONFIG_AFE4960)
zephyr_include_directories(./include)

zephyr_library()
zephyr_library_sources(
  src/afe4960.c
)
endif()

