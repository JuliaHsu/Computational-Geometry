set(LIB_SRC
    polygon.cpp
)

include_directories(
    "${CMAKE_SOURCE_DIR}/src"
)

add_library(polygon ${LIB_SRC})
