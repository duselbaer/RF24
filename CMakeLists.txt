# Some Platform Checks first
INCLUDE (CheckIncludeFiles)
CHECK_INCLUDE_FILES (endian.h HAVE_ENDIAN_H)
CHECK_INCLUDE_FILES (machine/endian.h HAVE_MACHINE_ENDIAN_H)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/NRF24L01/v1/config.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(
  RF24 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/NRF24L01/v1/config.h

    ${CMAKE_CURRENT_SOURCE_DIR}/src/NRF24L01/v1/NRF24L01.cpp
)

add_library(RF24_MSP430 STATIC
  src/NRF24L01/v1/driver/port2_interrupt.cpp
)
