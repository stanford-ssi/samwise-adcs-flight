set(PICO_PLATFORM rp2350-arm-s)
# If this is not included it won't find the adcs.h header file
list(APPEND PICO_CONFIG_HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/adcs.h)
