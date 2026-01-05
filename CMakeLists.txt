cmake_minimum_required(VERSION 3.20)
project(nvse_vr VERSION 1.0.0 LANGUAGES CXX)

# C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 32-bit build (required for Fallout NV)
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "This project must be built as 32-bit (x86). Please use a 32-bit toolchain.")
endif()

# Output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Dependencies paths (adjust these to your environment)
set(NVSE_DIR "${CMAKE_SOURCE_DIR}/../nvse" CACHE PATH "Path to NVSE SDK")
set(OPENVR_DIR "${CMAKE_SOURCE_DIR}/../deps/openvr" CACHE PATH "Path to OpenVR SDK")
set(VULKAN_SDK_DIR "$ENV{VULKAN_SDK}" CACHE PATH "Path to Vulkan SDK")

# If env var isn't set, try common default install path (C:\VulkanSDK\<version>).
if((NOT VULKAN_SDK_DIR) OR (VULKAN_SDK_DIR STREQUAL "") OR (NOT EXISTS "${VULKAN_SDK_DIR}/Include/vulkan/vulkan.h"))
    file(GLOB _vulkan_candidates LIST_DIRECTORIES true "C:/VulkanSDK/*")
    if(_vulkan_candidates)
        list(SORT _vulkan_candidates COMPARE NATURAL ORDER DESCENDING)
        list(GET _vulkan_candidates 0 _vulkan_latest)
        if(EXISTS "${_vulkan_latest}/Include/vulkan/vulkan.h")
            set(VULKAN_SDK_DIR "${_vulkan_latest}" CACHE PATH "Path to Vulkan SDK" FORCE)
        endif()
    endif()
endif()
set(DETOURS_DIR "${CMAKE_SOURCE_DIR}/../deps/Detours" CACHE PATH "Path to Microsoft Detours")

# Source files
set(SOURCES
    src/vr_manager.cc
    src/render_hook_manager.cc
    src/main.cc

    # Minimal NVSE/common runtime support (IDebugLog, ASSERT, etc.)
    src/id_debug_log.cc

    # Detours (compile from source; prebuilt .lib is not present in this workspace)
    ${DETOURS_DIR}/detours.cpp
    ${DETOURS_DIR}/disasm.cpp
    ${DETOURS_DIR}/modules.cpp
    ${DETOURS_DIR}/image.cpp
    ${DETOURS_DIR}/creatwth.cpp
)

set(HEADERS
    include/nvse_vr/vr_manager.h
    include/nvse_vr/render_hook_manager.h
)

# Create DLL
add_library(nvse_plugin_vr SHARED ${SOURCES} ${HEADERS} exports.def)

# Include directories
target_include_directories(nvse_plugin_vr PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${NVSE_DIR}
    ${NVSE_DIR}/nvse
    ${OPENVR_DIR}/headers
    ${VULKAN_SDK_DIR}/Include
    ${DETOURS_DIR}
)

# Link libraries
target_link_libraries(nvse_plugin_vr PRIVATE
    ${OPENVR_DIR}/lib/win32/openvr_api.lib
    d3d9.lib
    d3d11.lib
    dxgi.lib
    user32.lib
    kernel32.lib
    psapi.lib
)

# Preprocessor definitions
target_compile_definitions(nvse_plugin_vr PRIVATE
    WIN32
    _WINDOWS
    _USRDLL
    NVSE_PLUGIN_VR_EXPORTS
    RUNTIME_VERSION=0x040020D0
    RUNTIME
)

# Compiler options
if(MSVC)
    target_compile_options(nvse_plugin_vr PRIVATE
        /W3                 # Warning level 3
        /WX-                # Don't treat warnings as errors (for now)
        /Zi                 # Generate debug info
        /MP                 # Multi-processor compilation
        /permissive-        # Standards conformance
    )
    
    # Release-specific flags
    target_compile_options(nvse_plugin_vr PRIVATE
        $<$<CONFIG:Release>:/O2>        # Optimize for speed
        $<$<CONFIG:Release>:/GL>        # Whole program optimization
    )
    
    # Linker options
    target_link_options(nvse_plugin_vr PRIVATE
        /SUBSYSTEM:WINDOWS
        /MACHINE:X86
        /DEBUG                          # Generate PDB
        $<$<CONFIG:Release>:/LTCG>      # Link-time code generation
    )
endif()

# Module definition file (for DLL exports)
set_target_properties(nvse_plugin_vr PROPERTIES
    PREFIX ""                           # No "lib" prefix
    SUFFIX ".dll"                       # Explicit .dll suffix
)

# Install target (optional - copies to game directory)
# Set FALLOUT_NV_PATH environment variable to auto-install
if(DEFINED ENV{FALLOUT_NV_PATH})
    install(TARGETS nvse_plugin_vr
        RUNTIME DESTINATION "$ENV{FALLOUT_NV_PATH}/Data/NVSE/Plugins"
    )
    message(STATUS "Install target configured for: $ENV{FALLOUT_NV_PATH}/Data/NVSE/Plugins")
endif()

# Copy OpenVR DLL to output directory
add_custom_command(TARGET nvse_plugin_vr POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENVR_DIR}/bin/win32/openvr_api.dll"
        $<TARGET_FILE_DIR:nvse_plugin_vr>
    COMMENT "Copying openvr_api.dll to output directory"
)
