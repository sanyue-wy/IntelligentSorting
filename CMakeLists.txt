cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME IntelligentSorting)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/Middleware/Driver/drv_can.c
    User/Middleware/Algorithm/Controller/pid.c
    User/Middleware/Algorithm/Controller/feedforward.c
    User/Middleware/Algorithm/Controller/td.c
    User/Device/Tool/Debug_vofa/vofa.c
    User/Device/StepMotor/stepmotor_ZDT.c
    User/Device/StepMotor/stepmotor.c
    User/Device/DJ_Motor/dji_3508.c
    User/Device/servo/servo.c
     User/Middleware/Driver/drv_uart.c
    User/Middleware/Driver/drv_pwm.c
    User/APP/debug.c
    User/APP/task_it.c
    User/APP/Module/motion_sys_XY.c
   User/Middleware/Driver/drv_tim.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/Middleware/Driver
    User/Middleware/Algorithm/Controller
    User/Device/Tool/Debug_vofa
    User/Device/StepMotor
    User/Device/DJ_Motor
    User/Device/servo
    User/APP
    User/APP/Module
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Copy compile_commands.json to the source root after every build,
# so IDE IntelliSense (VSCode C/C++, clangd, ...) can pick it up
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Syncing compile_commands.json to source root for IDE"
)
