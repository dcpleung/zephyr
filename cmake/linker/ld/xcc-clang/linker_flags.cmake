# XCC/Clang does not support linker flag -no-pie.
# If specified, it will complain about unused command arguments
# which results in build error if -Werror is enabled.
# So don't pass -no-pie.
set_property(TARGET linker PROPERTY no_position_independent)
