cmake_minimum_required(VERSION 3.10)

project(roboclaw_serial LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the header-only library
add_library(roboclaw_serial INTERFACE)

target_include_directories(roboclaw_serial INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the headers
install(DIRECTORY include/roboclaw_serial
    DESTINATION include
)

# Make the project importable from the install directory
install(TARGETS roboclaw_serial
    EXPORT roboclaw_serial_targets
    INCLUDES DESTINATION include
)

install(EXPORT roboclaw_serial_targets
    FILE roboclaw_serialConfig.cmake
    DESTINATION lib/cmake/roboclaw_serial
)

# Testing
include(CTest) # Provides option for BUILD_TESTING, defaulted to ON
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(test_roboclaw_serial
        test/test_main.cpp
    )

    target_link_libraries(test_roboclaw_serial
        PRIVATE
            GTest::gtest
            GTest::gtest_main
            roboclaw_serial
    )

    gtest_discover_tests(test_roboclaw_serial)
endif()