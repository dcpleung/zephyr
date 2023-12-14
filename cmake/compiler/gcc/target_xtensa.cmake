# SPDX-License-Identifier: Apache-2.0

if(CONFIG_XTENSA_WINDOWED_ABI)
  list(APPEND TOOLCHAIN_C_FLAGS -mabi=windowed)
  list(APPEND TOOLCHAIN_LD_FLAGS -mabi=windowed)
endif()
