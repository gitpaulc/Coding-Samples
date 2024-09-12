cmake_minimum_required(VERSION 3.8)
project(ComputationalBiology)

# Do not generate ZERO_BUILD project.

set(CMAKE_SUPPRESS_REGENERATION ON)

# Visual Studio: Win32 or x64?

set(PATH_64 "")
if (${CMAKE_VS_PLATFORM_NAME} STREQUAL "x64")
  set(PATH_64 "/x64")
else()
  message(FATAL_ERROR "Requires 64-bit configuration." )
endif()

# Require at least C++ 14.

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# CMake variables. Options to include various libraries.

set(OPEN_CV_VERSION_STR "")
set(USE_CUDA ON CACHE BOOL "Use CUDA? Required." FORCE)
if (${USE_CUDA})
  enable_language(CUDA)
  set(CUDA_VERBOSE_BUILD ON)
endif()

set(IMG_OUTPUT_FOLDER "../image_output" CACHE PATH "Where should image outputs be written?")
set(VIDEO_OUTPUT_FOLDER "../video_output" CACHE PATH "Where should video outputs be written?")

file(MAKE_DIRECTORY "${IMG_OUTPUT_FOLDER}")
file(MAKE_DIRECTORY "${VIDEO_OUTPUT_FOLDER}")

# Header files.

set(HEADER_FILES
  computational_biology.h
)

# Source files.

set(SOURCE_FILES
  computational_biology.cpp
)

# Binary output.

add_executable(ComputationalBiology ${HEADER_FILES} ${SOURCE_FILES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ComputationalBiology)

# Includes.

target_include_directories(ComputationalBiology PUBLIC ".")

# Compiler flags.

if (${USE_CUDA})
  set_source_files_properties(computational_biology.cpp PROPERTIES LANGUAGE CUDA)
endif()

# Linking.

# Binary dependencies.

# Preprocessor Definitions.

if (${USE_CUDA})
  add_definitions(-DUSE_CUDA)
endif()

add_definitions(-DIMG_OUTPUT_FOLDER="${IMG_OUTPUT_FOLDER}")
add_definitions(-DVIDEO_OUTPUT_FOLDER="${VIDEO_OUTPUT_FOLDER}")
