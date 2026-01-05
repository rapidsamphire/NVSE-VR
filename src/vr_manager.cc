// VrManager implementation - OpenVR initialization and device management.

#include "nvse_vr/vr_manager.h"

#include <cstring>

#include "nvse_vr/nvse_compat.h"

#include "nvse/PluginAPI.h"

// DXVK interop interfaces for D3D9-to-Vulkan texture sharing
#undef INTERFACE
#define INTERFACE ID3D9VkInteropDevice
DECLARE_INTERFACE_(ID3D9VkInteropDevice, IUnknown) {
  STDMETHOD_(void, GetVulkanHandles)(VkInstance* instance, VkPhysicalDevice* physical_device, VkDevice* device) PURE;
  STDMETHOD_(void, GetSubmissionQueue)(VkQueue* queue, uint32_t* queue_index, uint32_t* queue_family_index) PURE;
  STDMETHOD_(void, TransitionTextureLayout)(struct ID3D9VkInteropTexture* texture, const VkImageSubresourceRange* subresources, VkImageLayout old_layout, VkImageLayout new_layout) PURE;
  STDMETHOD_(void, FlushRenderingCommands)() PURE;
  STDMETHOD_(void, LockSubmissionQueue)() PURE;
  STDMETHOD_(void, ReleaseSubmissionQueue)() PURE;
  STDMETHOD_(void, LockDevice)() PURE;
  STDMETHOD_(void, UnlockDevice)() PURE;
};
#undef INTERFACE

#undef INTERFACE
#define INTERFACE ID3D9VkInteropTexture
DECLARE_INTERFACE_(ID3D9VkInteropTexture, IUnknown) {
  STDMETHOD(GetVulkanImageInfo)(VkImage* handle, VkImageLayout* layout, VkImageCreateInfo* info) PURE;
};
#undef INTERFACE

// DXVK interop interface GUIDs
static const GUID kIidId3d9VkInteropDevice = 
    {0x2eaa4b89, 0x0107, 0x4bdb, {0x87, 0xf7, 0x0f, 0x54, 0x1c, 0x49, 0x3c, 0xe0}};
static const GUID kIidId3d9VkInteropTexture = 
    {0xd56344f5, 0x8d35, 0x46fd, {0x80, 0x6d, 0x94, 0xc3, 0x51, 0xb4, 0x72, 0xc1}};

namespace nvse_vr {

// RAII wrapper for Vulkan submission queue lock
class VkSubmissionQueueLockGuard {
 public:
  explicit VkSubmissionQueueLockGuard(ID3D9VkInteropDevice* interop) : interop_(interop) {
    if (interop_) {
      interop_->LockSubmissionQueue();
    }
  }
  
  ~VkSubmissionQueueLockGuard() {
    if (interop_) {
      interop_->ReleaseSubmissionQueue();
    }
  }
  
  // Delete copy and move
  VkSubmissionQueueLockGuard(const VkSubmissionQueueLockGuard&) = delete;
  VkSubmissionQueueLockGuard& operator=(const VkSubmissionQueueLockGuard&) = delete;

 private:
  ID3D9VkInteropDevice* interop_;
};

VrManager& VrManager::GetInstance() {
  static VrManager instance;
  return instance;
}

bool VrManager::Initialize() {
  _MESSAGE("Initializing OpenVR...");
  
  // Check if SteamVR is available
  if (!vr::VR_IsHmdPresent()) {
    _MESSAGE("No HMD detected - will retry during gameplay");
    _MESSAGE("Make sure your headset is connected and SteamVR is running");
    return false;
  }
  
  if (!vr::VR_IsRuntimeInstalled()) {
    _ERROR("OpenVR runtime not installed!");
    return false;
  }
  
  // Initialize OpenVR
  vr::EVRInitError init_error = vr::VRInitError_None;
  vr_system_ = vr::VR_Init(&init_error, vr::VRApplication_Scene);
  
  if (init_error != vr::VRInitError_None) {
    _ERROR("VR_Init failed: %s", vr::VR_GetVRInitErrorAsEnglishDescription(init_error));
    return false;
  }
  
  if (!vr_system_) {
    _ERROR("VR_Init returned null system interface");
    return false;
  }
  
  _MESSAGE("OpenVR initialized successfully!");
  
  // Get compositor interface
  vr_compositor_ = vr::VRCompositor();
  if (!vr_compositor_) {
    _ERROR("Failed to get VRCompositor interface");
    vr::VR_Shutdown();
    vr_system_ = nullptr;
    return false;
  }
  
  _MESSAGE("VRCompositor interface acquired");
  
  // Log HMD info
  char model_number[256] = {};
  vr_system_->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, 
      vr::Prop_ModelNumber_String, model_number, sizeof(model_number));
  _MESSAGE("HMD: %s", model_number);
  
  // Get recommended render target size
  uint32_t width, height;
  GetRecommendedRenderTargetSize(&width, &height);
  _MESSAGE("Recommended render target size: %ux%u", width, height);
  
  return true;
}

void VrManager::Shutdown() {
  if (vr_system_) {
    vr::VR_Shutdown();
    vr_system_ = nullptr;
    vr_compositor_ = nullptr;
    _MESSAGE("OpenVR shutdown complete");
  }
  
  // Release eye surfaces
  for (int i = 0; i < 2; i++) {
    if (eye_surfaces_[i]) {
      eye_surfaces_[i]->Release();
      eye_surfaces_[i] = nullptr;
    }
    if (eye_depth_stencils_[i]) {
      eye_depth_stencils_[i]->Release();
      eye_depth_stencils_[i] = nullptr;
    }
    if (eye_textures_[i]) {
      eye_textures_[i]->Release();
      eye_textures_[i] = nullptr;
    }
  }
}

void VrManager::Update() {
  if (!vr_system_) {
    // Try to initialize if not ready
    static int retry_count = 0;
    if (++retry_count % 300 == 0) {
      _MESSAGE("Retrying OpenVR initialization... (attempt %d)", retry_count / 300);
      if (Initialize()) {
        _MESSAGE("OpenVR initialized on retry!");
      }
    }
    return;
  }
  
  // Initialize Vulkan interop if we have D3D9 device but not Vulkan handles
  if (d3d9_device_ && !vk_device_) {
    InitializeVulkanInterop();
  }
}

void VrManager::BeginFrame() {
  if (!vr_system_ || !vr_compositor_) {
    return;
  }
  
  vr_frame_count_++;
  
  if (vr_frame_count_ % 100 == 0) {
    _MESSAGE("BeginFrame: VR frame %u", vr_frame_count_);
  }
  
  // WaitGetPoses MUST be called on render thread for frames to display in HMD
  vr::EVRCompositorError err = vr_compositor_->WaitGetPoses(
      tracked_device_poses_, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
  
  if (err != vr::VRCompositorError_None) {
    _ERROR("WaitGetPoses failed on frame %u: %d", vr_frame_count_, err);
  }
  
  // Update HMD pose with fresh prediction (using seqlock pattern for thread safety)
  if (tracked_device_poses_[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
    const bool was_valid = hmd_pose_valid_;
    hmd_pose_seq_.fetch_add(1, std::memory_order_acq_rel);  // Begin write
    hmd_pose_ = tracked_device_poses_[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
    hmd_pose_seq_.fetch_add(1, std::memory_order_acq_rel);  // End write
    hmd_pose_valid_ = true;
    if (!was_valid) {
      _MESSAGE("VrManager: HMD pose valid (render thread).");
    }
  } else {
    hmd_pose_valid_ = false;
  }
}

bool VrManager::TryGetHmdPoseForCamera(vr::HmdMatrix34_t& out_pose) const {
  if (!hmd_pose_valid_) {
    return false;
  }

  // Seqlock pattern: retry if sequence number changes during read
  while (true) {
    uint32_t seq1 = hmd_pose_seq_.load(std::memory_order_acquire);
    if (seq1 & 1) {  // Odd = write in progress
      continue;
    }
    out_pose = hmd_pose_;
    uint32_t seq2 = hmd_pose_seq_.load(std::memory_order_acquire);
    if (seq1 == seq2) {  // No write occurred during read
      return true;
    }
  }
}

void VrManager::Present() {
  if (!vr_system_ || !vr_compositor_) {
    return;
  }
  
  if (vr_frame_count_ % 100 == 0) {
    _MESSAGE("Present: VR frame %u", vr_frame_count_);
  }
  
  // Submit eye textures to compositor
  SubmitEyeTextures();
  
  // Tell compositor to begin work immediately instead of waiting for next WaitGetPoses
  vr_compositor_->PostPresentHandoff();
  
  eyes_captured_[0] = false;
  eyes_captured_[1] = false;
}

bool VrManager::GetEyeViews(vr::HmdMatrix34_t& left_eye, vr::HmdMatrix34_t& right_eye) const {
  if (!vr_system_) {
    // Return identity matrices as mock data
    memset(&left_eye, 0, sizeof(left_eye));
    memset(&right_eye, 0, sizeof(right_eye));
    left_eye.m[0][0] = left_eye.m[1][1] = left_eye.m[2][2] = 1.0f;
    right_eye.m[0][0] = right_eye.m[1][1] = right_eye.m[2][2] = 1.0f;
    
    // Mock IPD offset
    left_eye.m[0][3] = -0.032f;
    right_eye.m[0][3] = 0.032f;
    return true;
  }
  
  left_eye = vr_system_->GetEyeToHeadTransform(vr::Eye_Left);
  right_eye = vr_system_->GetEyeToHeadTransform(vr::Eye_Right);
  return true;
}

vr::HmdMatrix44_t VrManager::GetProjectionMatrix(vr::EVREye eye, float near_z, float far_z) const {
  if (!vr_system_) {
    vr::HmdMatrix44_t identity{};
    identity.m[0][0] = identity.m[1][1] = identity.m[2][2] = identity.m[3][3] = 1.0f;
    return identity;
  }
  return vr_system_->GetProjectionMatrix(eye, near_z, far_z);
}

vr::HmdMatrix34_t VrManager::GetEyeToHeadTransform(vr::EVREye eye) const {
  if (!vr_system_) {
    vr::HmdMatrix34_t identity{};
    identity.m[0][0] = identity.m[1][1] = identity.m[2][2] = 1.0f;
    return identity;
  }
  return vr_system_->GetEyeToHeadTransform(eye);
}

float VrManager::GetIpd() const {
  if (!vr_system_) {
    return 0.063f;  // Default 63mm
  }
  
  // Get IPD from SteamVR user settings
  vr::ETrackedPropertyError error;
  float ipd = vr_system_->GetFloatTrackedDeviceProperty(
      vr::k_unTrackedDeviceIndex_Hmd,
      vr::Prop_UserIpdMeters_Float,
      &error);
  
  if (error == vr::TrackedProp_Success) {
    return ipd;
  }
  
  return 0.063f;  // Fallback
}

void VrManager::GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height) const {
  if (vr_system_) {
    vr_system_->GetRecommendedRenderTargetSize(width, height);
  } else {
    *width = 1512;   // Fallback resolution
    *height = 1680;
  }
}

void VrManager::SetD3d9Device(IDirect3DDevice9* device) {
  if (!device || d3d9_device_) return;
  
  d3d9_device_ = device;
  _MESSAGE("VrManager: D3D9 device set: %p", device);
  
  // Get adapter info
  D3DADAPTER_IDENTIFIER9 adapter_id{};
  IDirect3D9* d3d9 = nullptr;
  if (SUCCEEDED(device->GetDirect3D(&d3d9)) && d3d9) {
    UINT adapter = D3DADAPTER_DEFAULT;
    if (SUCCEEDED(d3d9->GetAdapterIdentifier(adapter, 0, &adapter_id))) {
      _MESSAGE("D3D9 Adapter: %s", adapter_id.Description);
    }
    d3d9->Release();
  }
  
  // Initialize Vulkan interop now that we have D3D9 device
  if (vr_system_ && !vk_device_) {
    InitializeVulkanInterop();
  }
  
  if (vr_system_ && !eye_textures_[0]) {
    CreateEyeTextures();
  }
}

void VrManager::CaptureEyeTexture(int eye, IDirect3DSurface9* render_target) {
  if (eye < 0 || eye > 1 || !render_target || !d3d9_device_) {
    static bool logged_once = false;
    if (!logged_once) {
      _MESSAGE("CaptureEyeTexture early return: eye=%d, renderTarget=%p, device=%p", 
               eye, render_target, d3d9_device_);
      logged_once = true;
    }
    return;
  }
  
  // Create eye textures if not yet created
  if (!eye_textures_[0]) {
    _MESSAGE("Creating eye textures on first capture...");
    if (!CreateEyeTextures()) {
      _ERROR("Failed to create eye textures for VR submission!");
      return;
    }
  }
  
  // Get surface from our texture
  IDirect3DSurface9* dest_surface = nullptr;
  if (FAILED(eye_textures_[eye]->GetSurfaceLevel(0, &dest_surface))) {
    _ERROR("Failed to get surface level from eye texture %d", eye);
    return;
  }
  
  // Copy from render target to our texture
  HRESULT hr = d3d9_device_->StretchRect(render_target, nullptr, dest_surface, nullptr, D3DTEXF_LINEAR);
  dest_surface->Release();
  
  if (SUCCEEDED(hr)) {
    eyes_captured_[eye] = true;
    static int capture_count = 0;
    if (++capture_count % 1000 == 0) {
      _MESSAGE("Successfully captured %d eye frames", capture_count);
    }
  } else {
    static int error_count = 0;
    if (++error_count <= 5) {
      _ERROR("StretchRect failed for eye %d: 0x%08X", eye, hr);
    }
  }
}

bool VrManager::CreateEyeTextures() {
  if (!d3d9_device_) {
    _ERROR("CreateEyeTextures: No D3D9 device!");
    return false;
  }
  
  // Get recommended render target size
  uint32_t width, height;
  GetRecommendedRenderTargetSize(&width, &height);
  
  _MESSAGE("Creating eye textures at %ux%u for VR submission", width, height);
  
  for (int eye = 0; eye < 2; eye++) {
    // Create render target texture
    HRESULT hr = d3d9_device_->CreateTexture(
        width, height, 1,
        D3DUSAGE_RENDERTARGET,
        D3DFMT_A8R8G8B8,
        D3DPOOL_DEFAULT,
        &eye_textures_[eye],
        nullptr);
    
    if (FAILED(hr)) {
      _ERROR("Failed to create eye texture %d: 0x%08X", eye, hr);
      return false;
    }
    
    hr = eye_textures_[eye]->GetSurfaceLevel(0, &eye_surfaces_[eye]);
    if (FAILED(hr)) {
      _ERROR("Failed to get eye surface %d: 0x%08X", eye, hr);
      return false;
    }
    
    // Create depth stencil surface
    hr = d3d9_device_->CreateDepthStencilSurface(
        width, height,
        D3DFMT_D24S8,
        D3DMULTISAMPLE_NONE, 0,
        TRUE,  // Discard
        &eye_depth_stencils_[eye],
        nullptr);
    
    if (FAILED(hr)) {
      _ERROR("Failed to create eye depth stencil %d: 0x%08X", eye, hr);
      return false;
    }
    
    _MESSAGE("Created eye %d: texture=%p, surface=%p, depth=%p (%ux%u)", 
             eye, eye_textures_[eye], eye_surfaces_[eye], eye_depth_stencils_[eye], width, height);
  }
  
  eye_texture_width_ = width;
  eye_texture_height_ = height;
  _MESSAGE("Eye textures created successfully! Dimensions: %ux%u", eye_texture_width_, eye_texture_height_);
  return true;
}

bool VrManager::InitializeVulkanInterop() {
  if (!d3d9_device_) {
    _MESSAGE("Cannot initialize Vulkan interop - no D3D9 device");
    return false;
  }
  
  // Query for ID3D9VkInteropDevice interface from D3D9 device
  ID3D9VkInteropDevice* vk_interop = nullptr;
  HRESULT hr = d3d9_device_->QueryInterface(kIidId3d9VkInteropDevice, reinterpret_cast<void**>(&vk_interop));
  if (FAILED(hr)) {
    _MESSAGE("Failed to query ID3D9VkInteropDevice - are you not using DXVK?");
    return false;
  }
  
  _MESSAGE("Successfully queried ID3D9VkInteropDevice from D3D9 device!");
  
  vk_interop->GetVulkanHandles(&vk_instance_, &vk_physical_device_, &vk_device_);
  vk_interop->GetSubmissionQueue(&vk_queue_, nullptr, &vk_queue_family_index_);
  vk_interop->Release();
  
  if (!vk_instance_ || !vk_physical_device_ || !vk_device_) {
    _ERROR("Failed to get Vulkan handles from DXVK");
    return false;
  }
  
  _MESSAGE("Retrieved Vulkan handles from DXVK:");
  _MESSAGE("  VkInstance: %p", vk_instance_);
  _MESSAGE("  VkPhysicalDevice: %p", vk_physical_device_);
  _MESSAGE("  VkDevice: %p", vk_device_);
  _MESSAGE("  VkQueue: %p (family %u)", vk_queue_, vk_queue_family_index_);
  
  return true;
}

void VrManager::SubmitEyeTextures() {
  if (!vr_compositor_ || !d3d9_device_) {
    return;
  }
  
  // Check if we have textures to submit
  if (!eye_textures_[0] || !eye_textures_[1]) {
    return;
  }
  
  // Get DXVK interop
  ID3D9VkInteropDevice* vk_interop = nullptr;
  if (FAILED(d3d9_device_->QueryInterface(kIidId3d9VkInteropDevice, reinterpret_cast<void**>(&vk_interop)))) {
    return;
  }
  
  VkSubmissionQueueLockGuard lock(vk_interop);
  
  vk_interop->FlushRenderingCommands();
  
  for (int eye = 0; eye < 2; eye++) {
    if (!eyes_captured_[eye] || !eye_textures_[eye]) {
      continue;
    }
    
    // Get Vulkan image info from the texture
    ID3D9VkInteropTexture* tex_interop = nullptr;
    HRESULT hr = eye_textures_[eye]->QueryInterface(kIidId3d9VkInteropTexture, reinterpret_cast<void**>(&tex_interop));
    if (FAILED(hr)) {
      static int error_count[2] = {0, 0};
      if (++error_count[eye] <= 3) {
        _ERROR("Failed to get ID3D9VkInteropTexture for eye %d: 0x%08X", eye, hr);
      }
      continue;
    }
    
    VkImage vk_image = VK_NULL_HANDLE;
    VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkImageCreateInfo image_info{};
    
    hr = tex_interop->GetVulkanImageInfo(&vk_image, &layout, &image_info);
    tex_interop->Release();
    
    // Check if we got a valid VkImage regardless of HRESULT
    if (vk_image == VK_NULL_HANDLE) {
      static int error_count[2] = {0, 0};
      if (++error_count[eye] <= 3) {
        _ERROR("GetVulkanImageInfo returned null VkImage for eye %d: hr=0x%08X", eye, hr);
      }
      continue;
    }
    
    // ALWAYS use our stored dimensions (DXVK's VkImageCreateInfo may return garbage)
    uint32_t width = eye_texture_width_;
    uint32_t height = eye_texture_height_;
    VkFormat format = VK_FORMAT_B8G8R8A8_UNORM;  // We created with D3DFMT_A8R8G8B8
    
    static int log_count = 0;
    if (++log_count <= 10 || log_count % 500 == 0) {
      _MESSAGE("Eye %d: VkImage=%p, submitting %ux%u", eye, vk_image, width, height);
    }
    
    // Create OpenVR Vulkan texture descriptor
    vr::VRVulkanTextureData_t vulkan_data{};
  #if defined(VK_USE_64_BIT_PTR_DEFINES) && (VK_USE_64_BIT_PTR_DEFINES != 0)
    vulkan_data.m_nImage = reinterpret_cast<uint64_t>(vk_image);
  #else
    vulkan_data.m_nImage = static_cast<uint64_t>(vk_image);
  #endif
    vulkan_data.m_pDevice = vk_device_;
    vulkan_data.m_pPhysicalDevice = vk_physical_device_;
    vulkan_data.m_pInstance = vk_instance_;
    vulkan_data.m_pQueue = vk_queue_;
    vulkan_data.m_nQueueFamilyIndex = vk_queue_family_index_;
    vulkan_data.m_nWidth = width;
    vulkan_data.m_nHeight = height;
    vulkan_data.m_nFormat = format;
    vulkan_data.m_nSampleCount = 1;
    
    vr::Texture_t vr_texture{};
    vr_texture.handle = &vulkan_data;
    vr_texture.eType = vr::TextureType_Vulkan;
    vr_texture.eColorSpace = vr::ColorSpace_Auto;
    
    // Submit to compositor
    vr::EVRCompositorError err = vr_compositor_->Submit(
        static_cast<vr::EVREye>(eye), 
        &vr_texture, 
        nullptr,  // Full texture bounds
        vr::Submit_Default);
    
    if (err != vr::VRCompositorError_None) {
      static int error_count = 0;
      if (++error_count <= 10 || error_count % 500 == 0) {
        _ERROR("VRCompositor::Submit failed for eye %d: %d", eye, err);
      }
    } else {
      static int submit_count = 0;
      if (++submit_count % 500 == 0) {
        _MESSAGE("Submitted %d eye textures to OpenVR compositor", submit_count);
      }
    }
  }
  
  vk_interop->Release();
}

IDirect3DSurface9* VrManager::GetEyeRenderTarget(int eye) {
  if (eye < 0 || eye > 1) return nullptr;
  EnsureEyeTexturesCreated();
  return eye_surfaces_[eye];
}

IDirect3DSurface9* VrManager::GetEyeDepthStencil(int eye) {
  if (eye < 0 || eye > 1) return nullptr;
  EnsureEyeTexturesCreated();
  return eye_depth_stencils_[eye];
}

bool VrManager::EnsureEyeTexturesCreated() {
  if (eye_textures_[0] && eye_textures_[1]) return true;
  return CreateEyeTextures();
}

}  // namespace nvse_vr
