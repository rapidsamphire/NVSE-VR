// VR Manager - OpenVR initialization and device management.
// This handles HMD tracking, eye texture creation, and compositor submission.

#ifndef NVSE_VR_VR_MANAGER_H_
#define NVSE_VR_VR_MANAGER_H_

#include <atomic>
#include <cstdint>

#include <d3d9.h>
#include <openvr.h>

// Prefer the real Vulkan SDK headers when installed.
#if defined(__has_include) && __has_include(<vulkan/vulkan.h>)
#include <vulkan/vulkan.h>
#else
#include "nvse_vr/vulkan_fwd.h"
#endif

namespace nvse_vr {

// Manages OpenVR runtime initialization, HMD pose tracking, and eye texture submission.
// This class implements a singleton pattern and is thread-safe for pose queries.
// 
// Usage:
//   VrManager::GetInstance().Initialize();
//   VrManager::GetInstance().BeginFrame();  // Call once per frame on render thread
//   VrManager::GetInstance().Present();     // Submit textures to compositor
class VrManager {
 public:
  // Returns the singleton instance
  static VrManager& GetInstance();

  // Initializes OpenVR runtime and compositor
  // Returns true if successful, false otherwise
  bool Initialize();
  
  // Shuts down OpenVR runtime
  void Shutdown();
  
  // Updates VR manager state (called from game loop)
  void Update();
  
  // Begins a new VR frame and updates poses (MUST be called on render thread)
  void BeginFrame();
  
  // Submits eye textures to OpenVR compositor and calls PostPresentHandoff
  void Present();

  // Returns whether OpenVR is initialized and ready
  bool IsInitialized() const { return vr_system_ != nullptr; }

  // Gets the current HMD pose matrix (device-to-tracking-space transform)
  vr::HmdMatrix34_t GetHmdPose() const { return hmd_pose_; }
  
  // Thread-safe pose query using seqlock pattern
  // Returns true if pose is valid, false otherwise
  bool TryGetHmdPoseForCamera(vr::HmdMatrix34_t& out_pose) const;

  // Gets eye-to-head transforms for both eyes
  bool GetEyeViews(vr::HmdMatrix34_t& left_eye, vr::HmdMatrix34_t& right_eye) const;

  // Gets OpenVR projection matrix for specified eye
  vr::HmdMatrix44_t GetProjectionMatrix(vr::EVREye eye, float near_z, float far_z) const;

  // Gets eye-to-head transform for specified eye
  vr::HmdMatrix34_t GetEyeToHeadTransform(vr::EVREye eye) const;

  // Gets inter-pupillary distance in meters
  float GetIpd() const;

  // Sets D3D9 device (called externally when device is discovered)
  void SetD3d9Device(IDirect3DDevice9* device);

  // Gets D3D9 device pointer
  IDirect3DDevice9* GetD3d9Device() const { return d3d9_device_; }

  // Captures eye render target to internal eye texture for submission
  void CaptureEyeTexture(int eye, IDirect3DSurface9* render_target);

  // Gets recommended render target dimensions from OpenVR
  void GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height) const;

  // Gets eye render target surface (for rendering)
  IDirect3DSurface9* GetEyeRenderTarget(int eye);
  
  // Gets eye depth/stencil surface (for rendering)
  IDirect3DSurface9* GetEyeDepthStencil(int eye);
  
  // Gets eye texture dimensions
  uint32_t GetEyeTextureWidth() const { return eye_texture_width_; }
  uint32_t GetEyeTextureHeight() const { return eye_texture_height_; }

  // Ensures eye textures are created (lazy initialization)
  bool EnsureEyeTexturesCreated();

  // Gets OpenVR system interface (for controller access)
  vr::IVRSystem* GetVrSystem() const { return vr_system_; }
  
  // Gets tracked device poses array (for controller tracking)
  const vr::TrackedDevicePose_t* GetDevicePoses() const { return tracked_device_poses_; }

  // Marks eye as captured (ready for submission)
  void MarkEyeCaptured(int eye) { 
    if (eye >= 0 && eye <= 1) eyes_captured_[eye] = true; 
  }

 private:
  VrManager() = default;
  ~VrManager() = default;
  
  // Delete copy and move constructors/assignment operators
  VrManager(const VrManager&) = delete;
  VrManager& operator=(const VrManager&) = delete;
  VrManager(VrManager&&) = delete;
  VrManager& operator=(VrManager&&) = delete;

  // Initializes Vulkan interop for DXVK texture sharing
  bool InitializeVulkanInterop();
  
  // Creates D3D9 eye render targets and depth buffers
  bool CreateEyeTextures();
  
  // Submits eye textures to OpenVR compositor via Vulkan
  void SubmitEyeTextures();

  // OpenVR interfaces
  vr::IVRSystem* vr_system_ = nullptr;
  vr::IVRCompositor* vr_compositor_ = nullptr;

  // HMD pose with seqlock for thread-safe reads
  vr::HmdMatrix34_t hmd_pose_{};
  vr::TrackedDevicePose_t tracked_device_poses_[vr::k_unMaxTrackedDeviceCount]{};
  std::atomic<uint32_t> hmd_pose_seq_{0};
  bool hmd_pose_valid_ = false;

  // D3D9 device (discovered externally)
  IDirect3DDevice9* d3d9_device_ = nullptr;

  // Vulkan interop handles (for DXVK texture submission)
  VkInstance vk_instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice vk_physical_device_ = VK_NULL_HANDLE;
  VkDevice vk_device_ = VK_NULL_HANDLE;
  VkQueue vk_queue_ = VK_NULL_HANDLE;
  uint32_t vk_queue_family_index_ = 0;

  // Eye textures (D3D9 render targets shared with OpenVR via Vulkan)
  IDirect3DTexture9* eye_textures_[2] = {nullptr, nullptr};
  IDirect3DSurface9* eye_surfaces_[2] = {nullptr, nullptr};
  IDirect3DSurface9* eye_depth_stencils_[2] = {nullptr, nullptr};
  bool eyes_captured_[2] = {false, false};
  uint32_t eye_texture_width_ = 0;
  uint32_t eye_texture_height_ = 0;

  // Frame counter for diagnostics
  uint32_t vr_frame_count_ = 0;
};

}  // namespace nvse_vr

#endif  // NVSE_VR_VR_MANAGER_H_
