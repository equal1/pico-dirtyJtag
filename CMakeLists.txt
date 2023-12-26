# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Release")
#set(TINYUSB_DEBUG_LEVEL 0)

# Define the two required variables before including
# the source code for watching a git repository.
set(PRE_CONFIGURE_FILE "git.c.in")
set(POST_CONFIGURE_FILE "${CMAKE_CURRENT_BINARY_DIR}/git.c")
include(git_watcher.cmake)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(dirtyJtag C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(dirtyJtag
		dirtyJtag.c
		usb_descriptors.c
		pio_jtag.c
        cdc_uart.c
		get_serial.c
		cmd.c
        led.c
)

target_include_directories(dirtyJtag PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

pico_generate_pio_header(dirtyJtag ${CMAKE_CURRENT_LIST_DIR}/jtag.pio)

pico_set_program_name(dirtyJtag "dirtyJtag")
pico_set_program_version(dirtyJtag "0.1")

#pico_enable_stdio_uart(dirtyJtag 0)
#pico_enable_stdio_usb(dirtyJtag 0)

# Create a library out of the compiled post-configure file.
add_library(cmake_git_version_tracking STATIC ${POST_CONFIGURE_FILE})

# Add the standard library to the build
target_link_libraries(dirtyJtag PRIVATE pico_stdlib)

# Add any user requested libraries
target_link_libraries(dirtyJtag PRIVATE
	cmake_git_version_tracking
    hardware_pio
	hardware_dma
	pico_unique_id
	tinyusb_device
	tinyusb_board
	pico_multicore
)

pico_add_extra_outputs(dirtyJtag)

