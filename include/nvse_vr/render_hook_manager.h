// Render Hook Manager - Single-pass stereoscopic rendering via D3D9 hooks.

#ifndef NVSE_VR_RENDER_HOOK_MANAGER_H_
#define NVSE_VR_RENDER_HOOK_MANAGER_H_

#include <d3d9.h>
#include <openvr.h>
#include <unordered_map>
#include <cstdint>

namespace nvse_vr {

class Main;
class BsRenderedTexture;

// Manages render hooks for stereoscopic VR rendering.
// This class intercepts the game's rendering pipeline and duplicates draw calls
// for left and right eyes using single-pass stereo rendering.
//
// Key features:
// - Hooks Main::Render to capture frame timing
// - Hooks DrawIndexedPrimitive/DrawPrimitive to duplicate geometry per eye
// - Hooks SetTransform to capture/override view and projection matrices
// - Hooks SetVertexShaderConstantF to inject per-eye view matrices into shaders
//
// Usage:
//   RenderHookManager::GetInstance().Initialize();
//   RenderHookManager::GetInstance().Update();  // Called from game loop
class RenderHookManager {
 public:
  // Eye rendering pass identifier
  enum class EyePass {
    kNone = 0,
    kLeft = 1,
    kRight = 2
  };

  // Eye render data (projection and transform from OpenVR)
  struct EyeRenderData {
    vr::HmdMatrix34_t eye_to_head;
    vr::HmdMatrix44_t projection;
    bool is_valid;
  };

  // Vertex shader matrix type detection
  enum class VsMatrixKind {
    kNone = 0,
    kView = 1,
    kViewProj = 2,
    kWorldViewProj = 3,
  };

  // Returns singleton instance
  static RenderHookManager& GetInstance();

  // Installs render hooks via Detours
  void Initialize();
  
  // Updates hook state (called from game loop)
  void Update();
  
  // Shuts down and removes hooks
  void Shutdown();

  // Gets current rendering eye pass
  EyePass GetCurrentEyePass() const { return current_eye_pass_; }
  
  // Sets current rendering eye pass
  void SetCurrentEyePass(EyePass pass) { current_eye_pass_ = pass; }

  // Gets eye render data
  const EyeRenderData& GetLeftEyeData() const { return left_eye_data_; }
  const EyeRenderData& GetRightEyeData() const { return right_eye_data_; }
  
  // Updates eye data from OpenVR (called per frame)
  void UpdateEyeData();

  // Helper functions for hooks
  void ApplyEyeOffsetToViewMatrix(D3DMATRIX* view_matrix);
  void ClearDepthBuffer();
  void ClearRenderTarget();
  void CaptureCurrentEye(int eye);

  // Hook function pointers (public for global hook function access)
  typedef void (__thiscall* RenderFunc)(Main*, BsRenderedTexture*, int, int);
  static RenderFunc original_render_;
  
  typedef void (__cdecl* ProcessImageSpaceShadersFunc)(void*, BsRenderedTexture*, BsRenderedTexture*);
  static ProcessImageSpaceShadersFunc original_process_image_space_shaders_;
  
  typedef void (__cdecl* CreateBackgroundTextureFunc)();
  static CreateBackgroundTextureFunc original_create_background_texture_;
  static CreateBackgroundTextureFunc original_pipboy_render_;
  static CreateBackgroundTextureFunc original_3d_menu_render_;

  typedef HRESULT (STDMETHODCALLTYPE* SetTransformFunc)(IDirect3DDevice9*, D3DTRANSFORMSTATETYPE, CONST D3DMATRIX*);
  static SetTransformFunc original_set_transform_;
  
  typedef HRESULT (STDMETHODCALLTYPE* DrawIndexedPrimitiveFunc)(IDirect3DDevice9*, D3DPRIMITIVETYPE, INT, UINT, UINT, UINT, UINT);
  static DrawIndexedPrimitiveFunc original_draw_indexed_primitive_;
  
  typedef HRESULT (STDMETHODCALLTYPE* DrawPrimitiveFunc)(IDirect3DDevice9*, D3DPRIMITIVETYPE, UINT, UINT);
  static DrawPrimitiveFunc original_draw_primitive_;

  typedef HRESULT (STDMETHODCALLTYPE* SetVertexShaderConstantFFunc)(IDirect3DDevice9*, UINT, CONST float*, UINT);
  static SetVertexShaderConstantFFunc original_set_vertex_shader_constant_f_;

  // D3D9 hook installation
  void HookD3d9SetTransform(IDirect3DDevice9* device);
  void HookD3d9DrawCalls(IDirect3DDevice9* device);
  void HookD3d9ShaderConstants(IDirect3DDevice9* device);

  // Stereo render target access (now uses VrManager's textures)
  IDirect3DSurface9* GetLeftEyeRt() const;
  IDirect3DSurface9* GetRightEyeRt() const;
  IDirect3DSurface9* GetLeftEyeDepth() const;
  IDirect3DSurface9* GetRightEyeDepth() const;
  uint32_t GetStereoWidth() const;
  uint32_t GetStereoHeight() const;

  // Configuration
  bool IsSinglePassStereoEnabled() const { return single_pass_stereo_enabled_; }
  void SetSinglePassStereoEnabled(bool enabled) { single_pass_stereo_enabled_ = enabled; }

  bool IsShaderConstantStereoEnabled() const { return shader_constant_stereo_enabled_; }
  void SetShaderConstantStereoEnabled(bool enabled) { shader_constant_stereo_enabled_ = enabled; }

  bool IsEyeMarkerEnabled() const { return eye_marker_enabled_; }
  void SetEyeMarkerEnabled(bool enabled) { eye_marker_enabled_ = enabled; }

  // Main render pass tracking
  bool IsInMainRenderPass() const { return in_main_render_pass_; }
  void SetInMainRenderPass(bool in_pass) { in_main_render_pass_ = in_pass; }
  void SetMainPassSurfaces(IDirect3DSurface9* rt0, IDirect3DSurface9* depth, const D3DVIEWPORT9& viewport);
  bool IsMainPassSurface(IDirect3DSurface9* rt0) const;

  // View matrix tracking
  void SetCurrentViewMatrix(const D3DMATRIX* matrix) { 
    if (matrix) current_view_matrix_ = *matrix; 
  }
  const D3DMATRIX& GetCurrentViewMatrix() const { return current_view_matrix_; }

  void SetCurrentProjMatrix(const D3DMATRIX* matrix) { 
    if (matrix) { 
      current_proj_matrix_ = *matrix; 
      has_proj_matrix_ = true; 
    } 
  }
  const D3DMATRIX& GetCurrentProjMatrix() const { return current_proj_matrix_; }
  bool HasCurrentProjMatrix() const { return has_proj_matrix_; }

  void SetCurrentWorldMatrix(const D3DMATRIX* matrix) { 
    if (matrix) { 
      current_world_matrix_ = *matrix; 
      has_world_matrix_ = true; 
    } 
  }
  const D3DMATRIX& GetCurrentWorldMatrix() const { return current_world_matrix_; }
  bool HasCurrentWorldMatrix() const { return has_world_matrix_; }

  // Shader constant tracking
  void NoteViewVsConstant(uint32_t start_register, bool transposed, const float* matrix16);
  void NoteViewProjVsConstant(uint32_t start_register, bool transposed, const float* matrix16);
  void NotePositionVsConstant(VsMatrixKind kind, uint32_t start_register, bool transposed, const float* matrix16, float diff);
  void NotePositionVsConstantForShader(IDirect3DVertexShader9* vs, VsMatrixKind kind, uint32_t start_register, bool transposed, float diff);
  bool GetPositionVsConstantForCurrentShader(IDirect3DDevice9* device, VsMatrixKind& out_kind, uint32_t& out_reg, bool& out_transposed) const;

  // Projection matrix utilities
  void CalculateProjectionMatrix(const vr::HmdMatrix44_t& vr_proj, float* out_matrix);
  bool GetEyeProjectionMatrix(EyePass eye, D3DMATRIX& out_matrix) const;

  // Per-frame reset
  void OnNewFrame();

  // Debug eye markers
  void MaybeDrawEyeMarker(IDirect3DDevice9* device, EyePass eye);
  void ForceStampEyeMarkers(IDirect3DDevice9* device);

 private:
  RenderHookManager() = default;
  ~RenderHookManager() = default;
  
  // Delete copy and move
  RenderHookManager(const RenderHookManager&) = delete;
  RenderHookManager& operator=(const RenderHookManager&) = delete;
  RenderHookManager(RenderHookManager&&) = delete;
  RenderHookManager& operator=(RenderHookManager&&) = delete;

  EyePass current_eye_pass_ = EyePass::kNone;
  EyeRenderData left_eye_data_{};
  EyeRenderData right_eye_data_{};
  IDirect3DDevice9* d3d9_device_ = nullptr;
  
  bool set_transform_hooked_ = false;
  bool draw_calls_hooked_ = false;
  bool shader_constants_hooked_ = false;
  bool single_pass_stereo_enabled_ = true;
  bool shader_constant_stereo_enabled_ = false;
  bool eye_marker_enabled_ = false;
  bool in_main_render_pass_ = false;

  D3DMATRIX current_view_matrix_{};
  D3DMATRIX current_proj_matrix_{};
  bool has_proj_matrix_ = false;
  D3DMATRIX current_world_matrix_{};
  bool has_world_matrix_ = false;

  D3DMATRIX left_eye_proj_matrix_{};
  D3DMATRIX right_eye_proj_matrix_{};
  bool has_eye_proj_matrices_ = false;

  // Main pass surface tracking
  IDirect3DSurface9* main_pass_rt0_ = nullptr;
  IDirect3DSurface9* main_pass_depth_ = nullptr;
  D3DVIEWPORT9 main_pass_viewport_{};

  // Shader constant tracking
  struct ShaderPosConstInfo {
    VsMatrixKind kind = VsMatrixKind::kNone;
    uint32_t reg = 0xFFFFFFFFu;
    bool transposed = false;
    float diff = 1e9f;
    uint32_t frame_id = 0;
  };
  std::unordered_map<IDirect3DVertexShader9*, ShaderPosConstInfo> shader_pos_consts_;
  uint32_t frame_id_ = 0;

  // View matrix constant tracking (optional debugging / legacy support).
  uint32_t view_vs_const_reg_ = 0xFFFFFFFFu;
  bool view_vs_const_transposed_ = false;
  bool last_view_vs_const_valid_ = false;
  float last_view_vs_const_[16]{};

  // View-projection matrix constant tracking.
  uint32_t view_proj_vs_const_reg_ = 0xFFFFFFFFu;
  bool view_proj_vs_const_transposed_ = false;
  bool last_view_proj_vs_const_valid_ = false;
  float last_view_proj_vs_const_[16]{};

  // Best-match position matrix constant tracking (per-frame heuristic).
  VsMatrixKind pos_vs_kind_ = VsMatrixKind::kNone;
  uint32_t pos_vs_const_reg_ = 0xFFFFFFFFu;
  bool pos_vs_const_transposed_ = false;
  bool last_pos_vs_const_valid_ = false;
  float last_pos_vs_const_[16]{};
  float pos_vs_best_diff_ = 1e9f;

  // Debug marker state
  bool eye_marker_drawn_left_ = false;
  bool eye_marker_drawn_right_ = false;
  uint32_t eye_marker_size_px_ = 256;
};

}  // namespace nvse_vr

#endif  // NVSE_VR_RENDER_HOOK_MANAGER_H_
