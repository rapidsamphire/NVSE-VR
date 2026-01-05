#pragma once

// Minimal Vulkan forward declarations.
// This project only needs opaque handles and a few struct types for the DXVK
// D3D9-to-Vulkan interop interfaces. Avoid requiring the full Vulkan SDK.

#include <cstdint>

typedef struct VkInstance_T* VkInstance;
typedef struct VkPhysicalDevice_T* VkPhysicalDevice;
typedef struct VkDevice_T* VkDevice;
typedef struct VkQueue_T* VkQueue;
typedef struct VkImage_T* VkImage;

// Vulkan enums are integer types ABI-wise; we only pass them through.
typedef std::int32_t VkImageLayout;

struct VkImageSubresourceRange;
struct VkImageCreateInfo;
