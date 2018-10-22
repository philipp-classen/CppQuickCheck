cmake_minimum_required(VERSION 2.6)
project(CppQuickCheck)
if(WIN32)
    set(CMAKE_CXX_FLAGS "/W4 /permissive- /EHsc")
else()
    set(CMAKE_CXX_FLAGS "-O3 -g -Wall -std=c++11")
endif()


add_library(cppqc SHARED src/Arbitrary.cpp)
target_include_directories(cppqc PUBLIC include)

option(CPPQC_USE_BOOST "Use Boost, allowing to build some of the examples." OFF)
if(CPPQC_USE_BOOST)
    find_package(Boost REQUIRED)
endif()

add_subdirectory(examples)

install(DIRECTORY "include/" DESTINATION "include"
    PATTERN ".*" EXCLUDE)
install(TARGETS cppqc DESTINATION "lib")

# "catch" based unit tests
enable_testing()
add_executable(
  all-catch-tests
  test/catch-main.cpp
  test/arbitrary-tests.cpp
  test/shrink-explosion-protection.cpp
  test/compact-check-tests.cpp
  test/functional-tests.cpp
)
target_link_libraries(all-catch-tests PRIVATE cppqc)
add_test(all-catch-tests all-catch-tests)

# workaround to force CMake to build test executable before running the test
# (source: http://stackoverflow.com/a/736838/783510)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS all-catch-tests)

# additional target to perform clang-format run, requires clang-format
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
        clangformat
        COMMAND /usr/bin/clang-format
        -style=file
        -i
        ${ALL_SOURCE_FILES}
)
