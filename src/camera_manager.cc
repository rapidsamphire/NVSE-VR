// CameraManager implementation - VR camera transform and scene graph integration.

#include "nvse_vr/camera_manager.h"

#include <cmath>
#include <cstring>

#include <windows.h>

#include <detours.h>

#include "nvse_vr/nvse_compat.h"
#include "nvse_vr/vr_manager.h"

#include "nvse/PluginAPI.h"

namespace nvse_vr {

namespace {

// ============================================================================
// Safe Memory Write Utilities
// ============================================================================

// Safely writes an 8-bit value to memory, handling page protection.
static void SafeWrite8(uint32_t addr, uint8_t data) {
  DWORD old_protect;
  VirtualProtect(reinterpret_cast<void*>(addr), 1, PAGE_EXECUTE_READWRITE, &old_protect);
  *reinterpret_cast<uint8_t*>(addr) = data;
  VirtualProtect(reinterpret_cast<void*>(addr), 1, old_protect, &old_protect);
}

// Safely writes a 32-bit value to memory, handling page protection.
static void SafeWrite32(uint32_t addr, uint32_t data) {
  DWORD old_protect;
  VirtualProtect(reinterpret_cast<void*>(addr), 4, PAGE_EXECUTE_READWRITE, &old_protect);
  *reinterpret_cast<uint32_t*>(addr) = data;
  VirtualProtect(reinterpret_cast<void*>(addr), 4, old_protect, &old_protect);
}

// Writes a relative jump instruction (5 bytes: E9 xx xx xx xx).
static void WriteRelJump(uint32_t jump_src, uint32_t jump_tgt) {
  SafeWrite8(jump_src, 0xE9);  // JMP opcode
  SafeWrite32(jump_src + 1, jump_tgt - jump_src - 5);  // Relative offset
}

// ============================================================================
// Game Addresses (FalloutNV 1.4.0.525)
// ============================================================================

// PlayerCharacter::UpdateCamera function address.
constexpr uint32_t kUpdateCameraAddr = 0x94AE40;

// Camera node capture hook location (inside GetActiveCameraNodeCall).
constexpr uint32_t kCameraHookAddr = 0x0094BDDA;

// NOTE: For inline assembly compatibility with MSVC, these must be 'const' 
// not 'constexpr'. The 'constexpr' keyword does not allocate memory that 
// inline asm can reference for jumps/calls.
const uint32_t kCameraReturnAddr = 0x0094BDDF;
const uint32_t kGetCameraNodeCallAddr = 0x00558310;

// Global camera pointers.
NiCameraLite** const kMainCameraPtr = reinterpret_cast<NiCameraLite**>(0x11E0760);
NiAVObjectLite** const kFirstPersonCameraNodePtr = reinterpret_cast<NiAVObjectLite**>(0x011E07D0);

// InterfaceManager for scene graph access.
constexpr uint32_t kInterfaceManagerPtr = 0x011D8A80;

// OSGlobals for first-person camera access.
constexpr uint32_t kOsGlobalsPtr = 0x011DEA0C;

// ============================================================================
// Minimal Game Structure Definitions
// ============================================================================

struct SceneGraphLite {
  uint8_t pad[0x0DC];
  NiCameraLite* camera;
};

struct InterfaceManagerLite {
  uint32_t flags;
  SceneGraphLite* scene_graph_004;
  SceneGraphLite* scene_graph_008;
};

struct OsGlobalsLite {
  uint8_t pad[0xA0];
  NiCameraLite* first_person_camera;
};

// ============================================================================
// Coordinate Conversion Constants
// ============================================================================

// Fallout NV uses ~69 game units per meter.
constexpr float kUnitsPerMeter = 69.0f;

// Distance threshold for detecting teleport/camera mode changes.
constexpr float kBaseTransformThreshold = 100.0f;
static float g_reference_hmd_height = 0.0f;
static bool g_reference_height_set = false;

// ============================================================================
// Helper Functions
// ============================================================================

// Safely checks if a pointer is readable.
static bool IsReadablePointer(const void* ptr, size_t size) {
  if (!ptr) {
    return false;
  }
  MEMORY_BASIC_INFORMATION mbi{};
  if (VirtualQuery(ptr, &mbi, sizeof(mbi)) != sizeof(mbi)) {
    return false;
  }
  if (mbi.State != MEM_COMMIT) {
    return false;
  }
  if ((mbi.Protect & PAGE_GUARD) || (mbi.Protect & PAGE_NOACCESS)) {
    return false;
  }
  const auto base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
  const auto end = base + mbi.RegionSize;
  const auto addr = reinterpret_cast<uintptr_t>(ptr);
  return addr >= base && (addr + size) <= end;
}

// Checks if a 3x3 matrix contains only finite values.
static bool IsFiniteMatrix(const NiMatrix33& m) {
  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(m.data[i])) {
      return false;
    }
  }
  return true;
}

// Checks if a vector contains only finite values.
static bool IsFiniteVector(const NiVector3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Gets InterfaceManager instance.
static InterfaceManagerLite* GetInterfaceManager() {
  return *reinterpret_cast<InterfaceManagerLite**>(kInterfaceManagerPtr);
}

// Gets OSGlobals instance.
static OsGlobalsLite* GetOsGlobals() {
  return *reinterpret_cast<OsGlobalsLite**>(kOsGlobalsPtr);
}

// Gets the active camera from scene graph.
static NiCameraLite* GetActiveSceneGraphCamera() {
  InterfaceManagerLite* ui = GetInterfaceManager();
  if (!ui) {
    return nullptr;
  }

  SceneGraphLite* sg_a = ui->scene_graph_004;
  SceneGraphLite* sg_b = ui->scene_graph_008;

  if (sg_a && sg_a->camera) {
    return sg_a->camera;
  }
  if (sg_b && sg_b->camera) {
    return sg_b->camera;
  }
  return nullptr;
}

// ============================================================================
// File-scope statics for inline assembly compatibility
// ============================================================================
// MSVC inline assembly has issues with C++ class static members due to name
// mangling. Use simple file-scope statics like the original nvse_plugin_vr.
static NiAVObjectLite* g_active_camera_node = nullptr;
static bool g_camera_node_hook_seen = false;

}  // namespace

CameraManager::UpdateCameraFunc CameraManager::original_update_camera_ = nullptr;
NiAVObjectLite* CameraManager::captured_camera_node_ = nullptr;
bool CameraManager::camera_node_captured_ = false;

static void OnCameraNodeCaptured(NiAVObjectLite* node) {
  if (!node || !IsReadablePointer(node, sizeof(*node))) {
    return;
  }

  // Update both the file-scope static (for asm) and class static (for API)
  g_camera_node_hook_seen = true;
  CameraManager::captured_camera_node_ = node;
  CameraManager::camera_node_captured_ = true;

  // Apply VR transform immediately when camera is captured (every frame).
  VrManager& vr = VrManager::GetInstance();
  if (vr.IsInitialized()) {
    vr::HmdMatrix34_t hmd_pose{};
    if (vr.TryGetHmdPoseForCamera(hmd_pose)) {
      CameraManager::GetInstance().ApplyVrTransform(hmd_pose);
    }
  }
}

static __declspec(naked) void HookCameraNodeAsm() {
  __asm {
    // Overwritten code: push 0; push 0; push ecx
    push 0
    push 0
    push ecx

    // Save all registers.
    pushad
    
    // Call GetCameraNode to get the active camera.
    mov ecx, [ebp-0x68]
    call kGetCameraNodeCallAddr
    
    // Store result in file-scope static (NOT class static!)
    mov g_active_camera_node, eax
    
    // Call our notification function.
    push eax
    call OnCameraNodeCaptured
    add esp, 4
    
    // Restore registers.
    popad

    // Jump back to original code flow.
    jmp kCameraReturnAddr
  }
}

CameraManager& CameraManager::GetInstance() {
  static CameraManager instance;
  return instance;
}

bool CameraManager::Initialize() {
  if (initialized_) {
    _MESSAGE("CameraManager: Already initialized");
    return true;
  }

  _MESSAGE("CameraManager: Initializing...");

  // Install UpdateCamera detour hook.
  original_update_camera_ = reinterpret_cast<UpdateCameraFunc>(kUpdateCameraAddr);
  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(reinterpret_cast<PVOID*>(&original_update_camera_),
               reinterpret_cast<PVOID>(HookUpdateCamera));
  LONG result = DetourTransactionCommit();

  if (result != NO_ERROR) {
    _ERROR("CameraManager: UpdateCamera hook failed: %ld", result);
    return false;
  }
  _MESSAGE("CameraManager: UpdateCamera hook installed");

  // Install inline camera node capture hook.
  WriteRelJump(kCameraHookAddr, reinterpret_cast<uint32_t>(HookCameraNodeAsm));
  _MESSAGE("CameraManager: Camera node capture hook installed");

  initialized_ = true;
  _MESSAGE("CameraManager: Initialization complete");
  return true;
}

void CameraManager::Shutdown() {
  if (!initialized_) {
    return;
  }

  // Remove UpdateCamera hook.
  if (original_update_camera_) {
    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourDetach(reinterpret_cast<PVOID*>(&original_update_camera_),
                 reinterpret_cast<PVOID>(HookUpdateCamera));
    DetourTransactionCommit();
    original_update_camera_ = nullptr;
  }

  initialized_ = false;
  _MESSAGE("CameraManager: Shutdown complete");
}

void CameraManager::RecalibrateHeight() {
  g_reference_height_set = false;
  _MESSAGE("CameraManager: Height recalibration requested - will capture on next frame");
}

NiAVObjectLite* CameraManager::ResolveActiveCamera() {
  static bool logged_choice = false;

  // Priority 1: Captured camera node from inline hook (use file-scope static).
  if (g_camera_node_hook_seen && g_active_camera_node && 
      IsReadablePointer(g_active_camera_node, sizeof(*g_active_camera_node))) {
    if (!logged_choice) {
      logged_choice = true;
      _MESSAGE("CameraManager: Using hooked camera node=%p", g_active_camera_node);
    }
    return g_active_camera_node;
  }

  // Priority 2: OSGlobals first-person camera.
  OsGlobalsLite* os = GetOsGlobals();
  if (os && os->first_person_camera && 
      IsReadablePointer(os->first_person_camera, sizeof(*os->first_person_camera))) {
    if (!logged_choice) {
      logged_choice = true;
      _MESSAGE("CameraManager: Using OSGlobals first-person camera=%p", 
               os->first_person_camera);
    }
    return reinterpret_cast<NiAVObjectLite*>(os->first_person_camera);
  }

  // Priority 3: First-person camera node global.
  NiAVObjectLite* fp_node = *kFirstPersonCameraNodePtr;
  if (fp_node && IsReadablePointer(fp_node, sizeof(*fp_node))) {
    if (!logged_choice) {
      logged_choice = true;
      _MESSAGE("CameraManager: Using g_1stPersonCameraNode=%p", fp_node);
    }
    return fp_node;
  }

  // Priority 4: Scene graph camera.
  NiCameraLite* scene_cam = GetActiveSceneGraphCamera();
  if (scene_cam && IsReadablePointer(scene_cam, sizeof(*scene_cam))) {
    if (!logged_choice) {
      logged_choice = true;
      _MESSAGE("CameraManager: Using SceneGraph camera=%p", scene_cam);
    }
    return reinterpret_cast<NiAVObjectLite*>(scene_cam);
  }

  // Priority 5: Main camera global.
  if (IsReadablePointer(kMainCameraPtr, sizeof(*kMainCameraPtr)) && *kMainCameraPtr &&
      IsReadablePointer(*kMainCameraPtr, sizeof(**kMainCameraPtr))) {
    if (!logged_choice) {
      logged_choice = true;
      _MESSAGE("CameraManager: Using g_spMainCamera=%p", *kMainCameraPtr);
    }
    return reinterpret_cast<NiAVObjectLite*>(*kMainCameraPtr);
  }

  return nullptr;
}

void CameraManager::ApplyVrTransform(const vr::HmdMatrix34_t& hmd_pose) {
  NiAVObjectLite* camera = ResolveActiveCamera();
  if (!camera) {
    return;
  }

  VrManager& vr = VrManager::GetInstance();
  if (!vr.IsInitialized()) {
    return;
  }

  // Get current camera transform.
  const NiTransform& current_transform = camera->local_transform;

  // Update base transform if invalid or if camera moved significantly (teleport).
  if (!base_transform_valid_) {
    base_transform_ = current_transform;
    base_transform_valid_ = true;
  } else {
    // Check for significant position change indicating teleport/level load.
    float dx = current_transform.translate.x - base_transform_.translate.x;
    float dy = current_transform.translate.y - base_transform_.translate.y;
    float dz = current_transform.translate.z - base_transform_.translate.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq > (kBaseTransformThreshold * kBaseTransformThreshold)) {
      base_transform_ = current_transform;
    }
  }

  // Validate base transform.
  if (!IsFiniteMatrix(base_transform_.rotate) || 
      !IsFiniteVector(base_transform_.translate)) {
    return;
  }

  ApplyVrTransformToNode(camera, hmd_pose, base_transform_);
}

void CameraManager::ApplyVrTransformToNode(NiAVObjectLite* camera,
                                            const vr::HmdMatrix34_t& hmd_pose,
                                            const NiTransform& base_transform) {
  if (!camera || !IsReadablePointer(camera, sizeof(*camera))) {
    return;
  }

  // ==========================================================================
  // COORDINATE SYSTEM CONVERSION
  // ==========================================================================
  // OpenVR:    X=right, Y=up,      Z=backward (toward user)
  // Gamebryo:  X=right, Y=forward, Z=up
  //
  // Conversion:
  //   FNV_X (right)   = OpenVR_X (right)
  //   FNV_Y (forward) = -OpenVR_Z (forward = -backward)
  //   FNV_Z (up)      = OpenVR_Y (up)
  // ==========================================================================

  // Extract OpenVR rotation matrix.
  const float r00 = hmd_pose.m[0][0], r01 = hmd_pose.m[0][1], r02 = hmd_pose.m[0][2];
  const float r10 = hmd_pose.m[1][0], r11 = hmd_pose.m[1][1], r12 = hmd_pose.m[1][2];
  const float r20 = hmd_pose.m[2][0], r21 = hmd_pose.m[2][1], r22 = hmd_pose.m[2][2];

  // Convert rotation matrix by rearranging columns.
  // [right, forward, up] from [right, up, backward]
  NiMatrix33 hmd_rot{};
  // Column 0 (right) = OpenVR column 0 (right)
  hmd_rot.data[0] = r00;   hmd_rot.data[3] = r10;   hmd_rot.data[6] = r20;
  // Column 1 (forward) = -OpenVR column 2 (backward)
  hmd_rot.data[1] = -r02;  hmd_rot.data[4] = -r12;  hmd_rot.data[7] = -r22;
  // Column 2 (up) = OpenVR column 1 (up)
  hmd_rot.data[2] = r01;   hmd_rot.data[5] = r11;   hmd_rot.data[8] = r21;

  // Extract and convert translation.
  // Note: OpenVR Y is up, which maps to Gamebryo Z.
  const float hmd_height_raw = hmd_pose.m[1][3];  // OpenVR Y (meters from floor)
  
  // Track reference height on first valid pose.
  // This is the user's standing eye height - we only want to apply deltas.
  if (!g_reference_height_set) {
    g_reference_hmd_height = hmd_height_raw;
    g_reference_height_set = true;
    _MESSAGE("CameraManager: Calibrated reference HMD height: %.3f m", g_reference_hmd_height);
  }
  
  // Calculate height delta from reference (crouch/jump/lean movements)
  const float height_delta = hmd_height_raw - g_reference_hmd_height;
  
  NiVector3 hmd_trans{};
  hmd_trans.x = hmd_pose.m[0][3] * kUnitsPerMeter;      // right = right
  hmd_trans.y = -hmd_pose.m[2][3] * kUnitsPerMeter;     // forward = -backward
  hmd_trans.z = height_delta * kUnitsPerMeter;          // up = height DELTA only

  // Validate converted values.
  if (!IsFiniteMatrix(hmd_rot) || !IsFiniteVector(hmd_trans)) {
    return;
  }

  // Combine base rotation with HMD rotation.
  const NiMatrix33 new_rot = MultiplyMatrix33(base_transform.rotate, hmd_rot);

  // Transform HMD position offset by base rotation.
  const NiVector3 offset = TransformVector(base_transform.rotate, hmd_trans);
  NiVector3 new_pos{};
  new_pos.x = base_transform.translate.x + offset.x;
  new_pos.y = base_transform.translate.y + offset.y;
  new_pos.z = base_transform.translate.z + offset.z;

  // Final validation.
  if (!IsFiniteMatrix(new_rot) || !IsFiniteVector(new_pos)) {
    return;
  }

  // Apply to camera.
  camera->local_transform.rotate = new_rot;
  camera->local_transform.translate = new_pos;
  UpdateWorldTransform(camera);

  // Periodic logging.
  static int log_counter = 0;
  if ((log_counter++ % 300) == 0) {
    _MESSAGE("CameraManager: VR pos=(%.1f,%.1f,%.1f) offset=(%.1f,%.1f,%.1f)",
             new_pos.x, new_pos.y, new_pos.z,
             hmd_trans.x, hmd_trans.y, hmd_trans.z);
  }
}

void CameraManager::UpdateWorldTransform(NiAVObjectLite* obj) {
  if (!IsReadablePointer(obj, sizeof(*obj))) {
    return;
  }

  if (obj->parent) {
    NiAVObjectLite* parent = obj->parent;
    if (!IsReadablePointer(parent, sizeof(*parent))) {
      return;
    }

    const NiMatrix33& parent_rot = parent->world_transform.rotate;
    const NiVector3& parent_pos = parent->world_transform.translate;

    obj->world_transform.rotate = MultiplyMatrix33(parent_rot, obj->local_transform.rotate);
    NiVector3 world_pos = TransformVector(parent_rot, obj->local_transform.translate);
    world_pos.x += parent_pos.x;
    world_pos.y += parent_pos.y;
    world_pos.z += parent_pos.z;
    obj->world_transform.translate = world_pos;
  } else {
    obj->world_transform.rotate = obj->local_transform.rotate;
    obj->world_transform.translate = obj->local_transform.translate;
  }
}

NiMatrix33 CameraManager::MultiplyMatrix33(const NiMatrix33& a, const NiMatrix33& b) {
  NiMatrix33 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float sum = 0.0f;
      for (int k = 0; k < 3; ++k) {
        sum += a.data[r * 3 + k] * b.data[k * 3 + c];
      }
      out.data[r * 3 + c] = sum;
    }
  }
  return out;
}

NiVector3 CameraManager::TransformVector(const NiMatrix33& m, const NiVector3& v) {
  NiVector3 out{};
  out.x = (m.data[0] * v.x) + (m.data[1] * v.y) + (m.data[2] * v.z);
  out.y = (m.data[3] * v.x) + (m.data[4] * v.y) + (m.data[5] * v.z);
  out.z = (m.data[6] * v.x) + (m.data[7] * v.y) + (m.data[8] * v.z);
  return out;
}

void __fastcall CameraManager::HookUpdateCamera(void* player, void* edx, 
                                                 uint8_t arg1, uint8_t arg2) {
  (void)edx;  // Unused, required for __fastcall.

  // When the game calls UpdateCamera, refresh the base transform for position.
  // This happens on camera mode changes, teleports, etc.
  // NOTE: We do NOT reset g_reference_height_set here - room-scale height
  // calibration should persist until explicit recalibration.
  CameraManager::GetInstance().ResetBaseTransform();

  // Call original function.
  if (original_update_camera_) {
    original_update_camera_(player, arg1, arg2);
  }

  if (!player) {
    return;
  }

  // Check if VR is available.
  VrManager& vr = VrManager::GetInstance();
  if (!vr.IsInitialized()) {
    static bool warned = false;
    if (!warned) {
      _MESSAGE("CameraManager: VR not initialized, skipping VR camera transforms");
      warned = true;
    }
    return;
  }

  // Apply VR transform.
  vr::HmdMatrix34_t hmd_pose{};
  if (vr.TryGetHmdPoseForCamera(hmd_pose)) {
    CameraManager::GetInstance().ApplyVrTransform(hmd_pose);
  }
}

}  // namespace nvse_vr
