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

set(HEADER_FILES
  ../complex_quadratic.h
  ../constant.h
  ../function.h
  ../number.h
  ../quadratic_number.h
  ../rational.h
)

# Source files.

set(SOURCE_FILES
  ../complex_quadratic.cpp
  ../constant.cpp
  ../function.cpp
  ../main.cpp
  ../number.cpp
  ../quadratic_number.cpp
  ../rational.cpp
)

# Binary output.

add_executable("${PROJ_NAME}" ${HEADER_FILES} ${SOURCE_FILES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT "${PROJ_NAME}")

# Includes.

# Compiler flags.

# Linking.

# Binary dependencies.

# Preprocessor Definitions.

