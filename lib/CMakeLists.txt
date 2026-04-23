# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CUSTOM custom)
add_subdirectory_ifdef(CONFIG_MATH math)
add_subdirectory_ifdef(CONFIG_FILTER filter)
add_subdirectory_ifdef(CONFIG_CONTROL control)
