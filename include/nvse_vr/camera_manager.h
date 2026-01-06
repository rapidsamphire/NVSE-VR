
#ifndef NVSE_VR_CAMERA_MANAGER_H_
#define NVSE_VR_CAMERA_MANAGER_H_

#include <cstdint>

#include <openvr.h>

namespace nvse_vr {
// Minimal NetImmerse/NiAVObject definitions to avoid full NVSE/NiTypes.h dependency.
struct NiVector3 {
  float x, y, z;
};

struct NiMatrix33 {
  float data[9];
};

struct NiTransform {
  NiMatrix33 rotate;
  NiVector3 translate;
};

struct NiAVObjectLite {
  void* vtable;                   // 000
  uint32_t ref_count;             // 004
  uint32_t unk008;                // 008
  uint32_t unk00C;                // 00C
  void* name;                     // 010
  void* extra_data;               // 014
  NiAVObjectLite* parent;         // 018
  uint32_t unk01C;                // 01C
  uint32_t unk020;                // 020
  uint32_t unk024;                // 024
  uint32_t unk028;                // 028
  float flt02C;                   // 02C
  uint32_t flags;                 // 030
  NiTransform local_transform;    // 034
  NiTransform world_transform;    // 064
};

// Minimal NiCamera layout (extends NiAVObject).
// Only fields we need for the amera manipulation.
struct NiCameraLite : public NiAVObjectLite {
  // 094-0EC: camera-specific padding
  uint8_t pad094[0x0EC - 0x094];
  // 0EC: frustum (l,r,t,b,n,f)
  float frustum_left;
  float frustum_right;
  float frustum_top;
  float frustum_bottom;
  float frustum_near;
  float frustum_far;
};

// Manages VR camera transforms by integrating HMD pose data with the game's
// scene graph camera system. This class intercepts the game's UpdateCamera
// function and applies VR head tracking transforms.
//
// Key features:
// - Hooks PlayerCharacter::UpdateCamera to refresh base transforms
// - Applies HMD rotation/translation relative to game camera base pose
// - Coordinate conversion from OpenVR (X-right, Y-up, Z-back) to
//   Gamebryo/FNV (X-right, Y-forward, Z-up)
// - Thread-safe base transform management
//
// Usage:
//   CameraManager::GetInstance().Initialize();  // Install hooks
//   // Per-frame updates happen automatically via hooks
class CameraManager {
 public:
  // Returns the singleton instance.
  static CameraManager& GetInstance();

  // Installs camera hooks. Call once during plugin initialization.
  // Returns true if hooks were successfully installed.
  bool Initialize();

  // Shuts down and removes hooks.
  void Shutdown();

  // Applies VR HMD transform to the active camera.
  // Called automatically from hooks or can be called manually.
  void ApplyVrTransform(const vr::HmdMatrix34_t& hmd_pose);

  // Returns whether the camera manager is initialized.
  bool IsInitialized() const { return initialized_; }

  // Resets base transform tracking (call on teleport, load screen, etc.)
  void ResetBaseTransform() { base_transform_valid_ = false; }

  // Recalibrates room-scale height reference to current HMD position.
  // Call this when the user wants to reset their "standing" reference height.
  void RecalibrateHeight();

  // Static members for inline hook access (public for free function access).
  // These are accessed by the inline assembly hook callback.
  static NiAVObjectLite* captured_camera_node_;
  static bool camera_node_captured_;

 private:
  CameraManager() = default;
  ~CameraManager() = default;

  // Delete copy and move.
  CameraManager(const CameraManager&) = delete;
  CameraManager& operator=(const CameraManager&) = delete;
  CameraManager(CameraManager&&) = delete;
  CameraManager& operator=(CameraManager&&) = delete;

  // Resolves the active camera from multiple possible sources.
  NiAVObjectLite* ResolveActiveCamera();

  // Applies VR transform to a specific camera node.
  void ApplyVrTransformToNode(NiAVObjectLite* camera,
                               const vr::HmdMatrix34_t& hmd_pose,
                               const NiTransform& base_transform);

  // Updates world transform from local transform and parent.
  static void UpdateWorldTransform(NiAVObjectLite* obj);

  // Matrix multiplication (3x3).
  static NiMatrix33 MultiplyMatrix33(const NiMatrix33& a, const NiMatrix33& b);

  // Transform a vector by a 3x3 matrix.
  static NiVector3 TransformVector(const NiMatrix33& m, const NiVector3& v);

  // Hook function types.
  using UpdateCameraFunc = void(__thiscall*)(void* player, uint8_t arg1, uint8_t arg2);

  // Hook trampolines (static for hook callbacks).
  static UpdateCameraFunc original_update_camera_;
  static void __fastcall HookUpdateCamera(void* player, void* edx, uint8_t arg1, uint8_t arg2);

  // State.
  bool initialized_ = false;
  NiTransform base_transform_{};
  bool base_transform_valid_ = false;
};

}  // namespace nvse_vr

#endif  // NVSE_VR_CAMERA_MANAGER_H_
