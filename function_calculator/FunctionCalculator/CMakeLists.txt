cmake_minimum_required(VERSION 3.8)
set(PROJ_NAME "FunctionCalculator")
project("${PROJ_NAME}")

# Do not generate ZERO_BUILD project.

set(CMAKE_SUPPRESS_REGENERATION ON)

# Visual Studio: Win32 or x64?

set(PATH_64 "")
if (${CMAKE_VS_PLATFORM_NAME} STREQUAL "x64")
  set(PATH_64 "/x64")
endif()

# Require at least C++ 14.

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Header files.

set(PRIVATE_HEADERS
  ../private_headers/dynamic_matrix.h
)

set(HEADER_FILES
  ../complex_quadratic.h
  ../fn_polynomial.h
  ../function.h
  ../mp_integer.h
  ../number.h
  ../pi_polynomial.h
  ../pi_rational.h
  ../quadratic_number.h
  ../rational.h
)

# Source files.

set(SOURCE_FILES
  ../complex_quadratic.cpp
  ../fn_polynomial.cpp
  ../function.cpp
  ../main.cpp
  ../mp_integer.cpp
  ../number.cpp
  ../pi_polynomial.cpp
  ../pi_rational.cpp
  ../quadratic_number.cpp
  ../rational.cpp
)

# Binary output.

add_executable("${PROJ_NAME}" ${PRIVATE_HEADERS} ${HEADER_FILES} ${SOURCE_FILES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT "${PROJ_NAME}")

# Includes.

target_include_directories("${PROJ_NAME}" PRIVATE ../private_headers)

# Compiler flags.

# Linking.

# Binary dependencies.

# Preprocessor Definitions.

