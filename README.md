# NVSE-VR

A New Vegas Script Extender (NVSE) plugin that adds Virtual Reality support to Fallout: New Vegas.

## Overview

NVSE-VR is a plugin that brings full VR support to Fallout: New Vegas using OpenVR and Vulkan interop. This rewrite utilizes modern rendering techniques including single-pass stereo rendering for optimal performance in VR.

### Features

- **OpenVR Integration**: Full support for SteamVR-compatible headsets
- **Vulkan Interop**: Uses DXVK for efficient D3D9-to-Vulkan translation
- **Single-Pass Stereo Rendering**: Optimized rendering for both eyes in a single pass
- **HMD Tracking**: Real-time head-mounted display pose tracking
- **Compositor Submission**: Direct texture submission to the VR compositor

## Requirements

### Runtime Requirements
- Fallout: New Vegas
- [NVSE (New Vegas Script Extender)](http://nvse.silverlock.org/)
- [SteamVR](https://store.steampowered.com/app/250820/SteamVR/)
- A VR headset compatible with OpenVR/SteamVR
- Windows operating system

### Build Requirements
- CMake 3.20 or higher
- C++20 compatible compiler (MSVC recommended)
- 32-bit (x86) build toolchain
- Vulkan SDK
- OpenVR SDK
- Microsoft Detours library
- NVSE SDK

## Building

### Setting Up Dependencies

1. **Vulkan SDK**: Download and install from [LunarG](https://vulkan.lunarg.com/)
   - The build script will auto-detect the Vulkan SDK from the `VULKAN_SDK` environment variable or default Windows installation path

2. **OpenVR SDK**: Clone or download from [OpenVR repository](https://github.com/ValveSoftware/openvr)

3. **NVSE SDK**: Download from [NVSE website](http://nvse.silverlock.org/)

4. **Microsoft Detours**: Clone from [Microsoft Detours repository](https://github.com/microsoft/Detours)

### CMake Configuration

The project expects dependencies in a specific layout. You can override paths using CMake cache variables:

```bash
cmake -B build -G "Visual Studio 17 2022" -A Win32 \
  -DNVSE_DIR="path/to/nvse" \
  -DOPENVR_DIR="path/to/openvr" \
  -DDETOURS_DIR="path/to/Detours" \
  -DVULKAN_SDK_DIR="path/to/VulkanSDK"
```

**Note**: This project requires a 32-bit (x86) build as Fallout: New Vegas is a 32-bit application.

### Building the Plugin

1. Generate build files:
   ```bash
   cmake -B build -G "Visual Studio 17 2022" -A Win32
   ```

2. Build the project:
   ```bash
   cmake --build build --config Release
   ```

3. The compiled `nvse_plugin_vr.dll` will be located in `build/bin/Release/`

### Auto-Install (Optional)

Set the `FALLOUT_NV_PATH` environment variable to your Fallout: New Vegas installation directory:

```bash
set FALLOUT_NV_PATH=C:\Program Files (x86)\Steam\steamapps\common\Fallout New Vegas
cmake --install build --config Release
```

This will automatically copy the plugin to `Data/NVSE/Plugins` in your game directory.

## Installation

1. Install [NVSE](http://nvse.silverlock.org/) if you haven't already
2. Create the directory structure: `<Fallout NV>/Data/NVSE/Plugins/`
3. Copy `nvse_plugin_vr.dll` to the `Plugins` folder
4. Copy `openvr_api.dll` from the OpenVR SDK to the same folder
5. Launch Fallout: New Vegas through NVSE (nvse_loader.exe)

## Usage

Once installed, the plugin will automatically:
- Initialize when NVSE loads the plugin
- Detect and connect to your VR headset via SteamVR
- Hook into the game's rendering pipeline
- Redirect rendering output to your VR headset

Make sure SteamVR is running before launching the game.

## Project Structure

```
NVSE-VR/
├── include/
│   ├── common/          # Common utilities (logging, errors, types)
│   └── nvse_vr/         # VR-specific headers
├── src/                 # Source files
│   ├── main.cc          # NVSE plugin entry point
│   ├── vr_manager.cc    # OpenVR runtime management
│   └── render_hook_manager.cc  # Rendering pipeline hooks
├── CMakeLists.txt       # Build configuration
├── exports.def          # DLL export definitions
└── LICENSE.MD           # GPL v3 license
```

## Architecture

The plugin consists of two main components:

1. **VrManager**: Handles OpenVR initialization, HMD tracking, and compositor submission
2. **RenderHookManager**: Intercepts D3D9 rendering calls and redirects them to Vulkan textures for VR display

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE.MD](LICENSE.MD) file for details.

## Acknowledgments

- [NVSE Team](http://nvse.silverlock.org/) for the New Vegas Script Extender
- [Valve](https://www.valvesoftware.com/) for OpenVR
- The Fallout: New Vegas modding community

## Disclaimer

This is an unofficial fan-made modification. Fallout: New Vegas is property of Bethesda Softworks and Obsidian Entertainment.
