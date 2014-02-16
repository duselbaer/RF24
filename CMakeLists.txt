include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(RF24 STATIC
  include/NRF24L01/v1/NRF24L01.hpp
)

# No Sources --> set link language manually
SET_TARGET_PROPERTIES(RF24 PROPERTIES LINKER_LANGUAGE C++)
