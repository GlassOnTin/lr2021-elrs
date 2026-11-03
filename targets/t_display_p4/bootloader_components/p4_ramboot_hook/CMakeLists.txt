idf_component_register(SRCS "hooks.c" REQUIRES bootloader_support)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u bootloader_include_custom_hooks")
