cmake_minimum_required(VERSION 3.10)

# Project name
project(TrifectaLinux_SerialReadTest)

# Specify the C and C++ standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add subdirectory for the DriverTrifectaLinux library
add_subdirectory(lib/Trifecta-Driver)

# Add the executable
add_executable(example_serial_read src/example_serial_read.c)

# Link to the DriverTrifectaLinux library
target_link_libraries(example_serial_read PRIVATE DriverTrifectaLinux)

# Include directories for the DriverTrifectaLinux library
target_include_directories(example_serial_read PRIVATE
    $<TARGET_PROPERTY:DriverTrifectaLinux,INTERFACE_INCLUDE_DIRECTORIES>
)
