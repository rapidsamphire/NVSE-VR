// RenderHookManager implementation - single-pass stereoscopic rendering via D3D9 hooks.

#include "nvse_vr/render_hook_manager.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include <psapi.h>
#include <windows.h>

#include <detours.h>

#include "nvse_vr/nvse_compat.h"

#include "nvse/PluginAPI.h"
#include "nvse/SafeWrite.h"

#include "nvse_vr/camera_manager.h"
#include "nvse_vr/vr_manager.h"

#pragma comment(lib, "psapi.lib")

namespace nvse_vr {

namespace {

// Game global D3D9 device pointer (FalloutNV 1.4.0.525).
IDirect3DDevice9** const kGameD3d9DevicePtr = reinterpret_cast<IDirect3DDevice9**>(0x11DA5B8);

// Main::Render hook state.
static bool g_frame_begun = false;

// Debug marker colors.
static constexpr D3DCOLOR kEyeMarkerLeftColor = D3DCOLOR_XRGB(255, 0, 0);
static constexpr D3DCOLOR kEyeMarkerRightColor = D3DCOLOR_XRGB(0, 255, 0);

static HRESULT STDMETHODCALLTYPE SetTransformHook(IDirect3DDevice9* device,
                                                 D3DTRANSFORMSTATETYPE state,
                                                 const D3DMATRIX* matrix);

static HRESULT STDMETHODCALLTYPE DrawIndexedPrimitiveHook(IDirect3DDevice9* device,
                                                         D3DPRIMITIVETYPE type,
                                                         INT base_vertex_index,
                                                         UINT min_vertex_index,
                                                         UINT num_vertices,
                                                         UINT start_index,
                                                         UINT prim_count);

static HRESULT STDMETHODCALLTYPE DrawPrimitiveHook(IDirect3DDevice9* device,
                                                  D3DPRIMITIVETYPE primitive_type,
                                                  UINT start_vertex,
                                                  UINT primitive_count);

static HRESULT STDMETHODCALLTYPE SetVertexShaderConstantFHook(IDirect3DDevice9* device,
                                                             UINT start_register,
                                                             const float* constant_data,
                                                             UINT vector4f_count);

static float MaxAbsDiff16RowMajor(const float* a, const float* b) {
  float max_diff = 0.0f;
  for (int i = 0; i < 16; ++i) {
    const float d = fabsf(a[i] - b[i]);
    if (d > max_diff) {
      max_diff = d;
    }
  }
  return max_diff;
}

static float MaxAbsDiff16Transpose(const float* a_row_major16, const D3DMATRIX& m) {
  float max_diff = 0.0f;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      const float d = fabsf(a_row_major16[r * 4 + c] - m.m[c][r]);
      if (d > max_diff) {
        max_diff = d;
      }
    }
  }
  return max_diff;
}

static void CopyD3dMatrixToFloat16(const D3DMATRIX& m, float out16[16], bool transpose) {
  if (!transpose) {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        out16[r * 4 + c] = m.m[r][c];
      }
    }
  } else {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        out16[r * 4 + c] = m.m[c][r];
      }
    }
  }
}

static D3DMATRIX MultiplyD3dMatrices(const D3DMATRIX& a, const D3DMATRIX& b) {
  D3DMATRIX out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      float sum = 0.0f;
      for (int k = 0; k < 4; ++k) {
        sum += a.m[r][k] * b.m[k][c];
      }
      out.m[r][c] = sum;
    }
  }
  return out;
}

static bool ApplyHmdPoseToViewMatrix(D3DMATRIX* view_matrix) {
  if (view_matrix == nullptr) {
    return false;
  }

  VrManager& vr = VrManager::GetInstance();
  vr::HmdMatrix34_t pose{};
  if (!vr.TryGetHmdPoseForCamera(pose)) {
    return false;
  }

  const float r00 = pose.m[0][0];
  const float r01 = pose.m[0][1];
  const float r02 = pose.m[0][2];
  const float r10 = pose.m[1][0];
  const float r11 = pose.m[1][1];
  const float r12 = pose.m[1][2];
  const float r20 = pose.m[2][0];
  const float r21 = pose.m[2][1];
  const float r22 = pose.m[2][2];

  // Fallout NV units: ~69 units per meter.
  constexpr float kUnitsPerMeter = 69.0f;
  const float t_x = pose.m[0][3] * kUnitsPerMeter;
  const float t_y = pose.m[1][3] * kUnitsPerMeter;
  const float t_z = pose.m[2][3] * kUnitsPerMeter;

  // Build an HMD view matrix (inverse of pose) and pre-multiply with game view.
  D3DMATRIX hmd_view{};
  hmd_view.m[0][0] = r00;
  hmd_view.m[0][1] = r10;
  hmd_view.m[0][2] = r20;
  hmd_view.m[0][3] = 0.0f;

  hmd_view.m[1][0] = r01;
  hmd_view.m[1][1] = r11;
  hmd_view.m[1][2] = r21;
  hmd_view.m[1][3] = 0.0f;

  hmd_view.m[2][0] = r02;
  hmd_view.m[2][1] = r12;
  hmd_view.m[2][2] = r22;
  hmd_view.m[2][3] = 0.0f;

  hmd_view.m[3][0] = -(r00 * t_x + r10 * t_y + r20 * t_z);
  hmd_view.m[3][1] = -(r01 * t_x + r11 * t_y + r21 * t_z);
  hmd_view.m[3][2] = -(r02 * t_x + r12 * t_y + r22 * t_z);
  hmd_view.m[3][3] = 1.0f;

  *view_matrix = MultiplyD3dMatrices(hmd_view, *view_matrix);
  return true;
}

static float BestMatchMatrix4x4(const float* constant_data16,
                                const D3DMATRIX& m,
                                bool* out_transposed) {
  const float diff_direct = MaxAbsDiff16RowMajor(constant_data16, &m.m[0][0]);
  const float diff_transpose = MaxAbsDiff16Transpose(constant_data16, m);

  if (diff_direct <= diff_transpose) {
    if (out_transposed) {
      *out_transposed = false;
    }
    return diff_direct;
  }

  if (out_transposed) {
    *out_transposed = true;
  }
  return diff_transpose;
}

// NOTE: We deliberately do not hook UI/menu rendering here yet.
// That will come when the UI system is implemented.

// Global hook function (not a member).
static void __fastcall MainRenderHook(Main* main,
                                     void* /*edx*/,
                                     BsRenderedTexture* texture,
                                     int arg2,
                                     int arg3) {
  static bool first_time = true;
  if (first_time) {
    _MESSAGE("Main::Render hook FIRST CALL");
    first_time = false;
  }

  static int call_count = 0;
  ++call_count;

  RenderHookManager& render_mgr = RenderHookManager::GetInstance();
  VrManager& vr = VrManager::GetInstance();

  // Call WaitGetPoses ONCE per frame.
  if (!g_frame_begun) {
    vr.BeginFrame();
    g_frame_begun = true;
    render_mgr.OnNewFrame();
  }

  // Attempt to capture D3D9 device from NiDX9Renderer singleton early.
  // This is intentionally the same as the original working implementation.
  static bool device_captured = false;
  if (!device_captured && call_count <= 10) {
    void** const renderer_singleton = reinterpret_cast<void**>(0x11C73B4);
    if (call_count == 1) {
      _MESSAGE("Attempting to capture D3D9 device from NiDX9Renderer singleton...");
    }

    if (renderer_singleton && *renderer_singleton) {
      const uintptr_t renderer_addr = reinterpret_cast<uintptr_t>(*renderer_singleton);

      HMODULE d3d9_module = GetModuleHandleA("d3d9.dll");
      MODULEINFO mod_info{};
      if (d3d9_module &&
          GetModuleInformation(GetCurrentProcess(), d3d9_module, &mod_info, sizeof(mod_info))) {
        const uintptr_t mod_start = reinterpret_cast<uintptr_t>(mod_info.lpBaseOfDll);
        const uintptr_t mod_end = mod_start + mod_info.SizeOfImage;

        for (size_t off = 0; off < 0x800; off += sizeof(void*)) {
          void* const candidate_ptr_loc = reinterpret_cast<void*>(renderer_addr + off);
          if (IsBadReadPtr(candidate_ptr_loc, sizeof(void*))) {
            continue;
          }

          IDirect3DDevice9* const dev = *reinterpret_cast<IDirect3DDevice9**>(candidate_ptr_loc);
          if (!dev || IsBadReadPtr(dev, sizeof(void*))) {
            continue;
          }

          if (IsBadReadPtr(dev, sizeof(void*) * 2)) {
            continue;
          }

          void** const vtbl = *reinterpret_cast<void***>(dev);
          if (!vtbl || IsBadReadPtr(vtbl, sizeof(void*))) {
            continue;
          }

          const uintptr_t entry = reinterpret_cast<uintptr_t>(vtbl[0]);
          if (entry >= mod_start && entry < mod_end) {
            _MESSAGE("Found D3D9 device via NiDX9Renderer scan at offset 0x%zx: %p", off, dev);
            vr.SetD3d9Device(dev);
            render_mgr.HookD3d9SetTransform(dev);
            device_captured = true;
            break;
          }
        }
      }

      if (!device_captured && call_count == 1) {
        _MESSAGE("NiDX9Renderer exists but device not yet located via scan");
      }
    } else if (call_count == 1) {
      _MESSAGE("NiDX9Renderer singleton not yet available at 0x11C73B4");
    }

    if (!device_captured && call_count == 10) {
      _MESSAGE("WARNING: Could not capture D3D9 device after 10 frames");
    }
  }

  if (call_count % 500 == 0) {
    _MESSAGE("Main::Render called %d times", call_count);
  }

  const auto original_render = RenderHookManager::original_render_;
  if (!original_render) {
    _ERROR("original_render_ is NULL!");
    return;
  }

  if (!vr.IsInitialized()) {
    original_render(main, texture, arg2, arg3);
    return;
  }

  render_mgr.UpdateEyeData();

  const auto& left_eye = render_mgr.GetLeftEyeData();
  const auto& right_eye = render_mgr.GetRightEyeData();
  if (!left_eye.is_valid || !right_eye.is_valid) {
    original_render(main, texture, arg2, arg3);
    return;
  }

  // Let VrManager do any non-render-thread updates.
  vr.Update();

  render_mgr.SetInMainRenderPass(true);

  IDirect3DDevice9* const device = vr.GetD3d9Device();
  if (device) {
    IDirect3DSurface9* rt0 = nullptr;
    IDirect3DSurface9* depth = nullptr;
    D3DVIEWPORT9 vp{};
    if (SUCCEEDED(device->GetRenderTarget(0, &rt0)) &&
        SUCCEEDED(device->GetDepthStencilSurface(&depth)) &&
        SUCCEEDED(device->GetViewport(&vp))) {
      render_mgr.SetMainPassSurfaces(rt0, depth, vp);
    }
    if (rt0) {
      rt0->Release();
    }
    if (depth) {
      depth->Release();
    }
  }

  // Single render call; draw hooks duplicate to both eyes.
  original_render(main, texture, arg2, arg3);

  render_mgr.SetInMainRenderPass(false);

  if (device && render_mgr.IsEyeMarkerEnabled()) {
    render_mgr.ForceStampEyeMarkers(device);
  }

  // Mirror left eye to desktop backbuffer.
  if (device && render_mgr.GetLeftEyeRt()) {
    IDirect3DSurface9* back_buffer = nullptr;
    if (SUCCEEDED(device->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &back_buffer)) &&
        back_buffer) {
      device->StretchRect(render_mgr.GetLeftEyeRt(), nullptr, back_buffer, nullptr, D3DTEXF_LINEAR);
      back_buffer->Release();
    }
  }

  // Mark eyes captured for submission.
  render_mgr.CaptureCurrentEye(0);
  render_mgr.CaptureCurrentEye(1);

  vr.Present();

  // Next frame BeginFrame should run again.
  g_frame_begun = false;
}

static HRESULT STDMETHODCALLTYPE SetTransformHook(IDirect3DDevice9* device,
                                                 D3DTRANSFORMSTATETYPE state,
                                                 const D3DMATRIX* matrix) {
  RenderHookManager& mgr = RenderHookManager::GetInstance();

  if ((state == D3DTS_VIEW || state == D3DTS_PROJECTION || state == D3DTS_WORLD) && matrix) {
    if (state == D3DTS_VIEW) {
      mgr.SetCurrentViewMatrix(matrix);
    } else if (state == D3DTS_PROJECTION) {
      mgr.SetCurrentProjMatrix(matrix);
    } else if (state == D3DTS_WORLD) {
      mgr.SetCurrentWorldMatrix(matrix);
    }

    // Override projection matrix with VR headset FOV during main pass.
    if (state == D3DTS_PROJECTION && mgr.IsInMainRenderPass()) {
      const RenderHookManager::EyePass eye_pass = mgr.GetCurrentEyePass();
      const RenderHookManager::EyeRenderData* eye_data = nullptr;

      if (eye_pass == RenderHookManager::EyePass::kLeft) {
        eye_data = &mgr.GetLeftEyeData();
      } else if (eye_pass == RenderHookManager::EyePass::kRight) {
        eye_data = &mgr.GetRightEyeData();
      } else {
        eye_data = &mgr.GetLeftEyeData();
      }

      if (eye_data && eye_data->is_valid) {
        D3DMATRIX vr_proj{};
        mgr.CalculateProjectionMatrix(eye_data->projection, reinterpret_cast<float*>(&vr_proj));
        return RenderHookManager::original_set_transform_(device, state, &vr_proj);
      }
    }

    // In single-pass mode, we do not modify the view here; draw hooks handle per-eye.
    if (state == D3DTS_VIEW && mgr.IsSinglePassStereoEnabled() && mgr.IsInMainRenderPass()) {
      return RenderHookManager::original_set_transform_(device, state, matrix);
    }

    // Legacy dual-pass eye offset support.
    if (state == D3DTS_VIEW && mgr.GetCurrentEyePass() != RenderHookManager::EyePass::kNone) {
      D3DMATRIX modified = *matrix;
      mgr.ApplyEyeOffsetToViewMatrix(&modified);
      return RenderHookManager::original_set_transform_(device, state, &modified);
    }
  }

  return RenderHookManager::original_set_transform_(device, state, matrix);
}

static HRESULT STDMETHODCALLTYPE SetVertexShaderConstantFHook(IDirect3DDevice9* device,
                                                             UINT start_register,
                                                             const float* constant_data,
                                                             UINT vector4f_count) {
  RenderHookManager& mgr = RenderHookManager::GetInstance();

  if (!mgr.IsShaderConstantStereoEnabled()) {
    return RenderHookManager::original_set_vertex_shader_constant_f_(
        device, start_register, constant_data, vector4f_count);
  }

  if (constant_data && vector4f_count == 4) {
    constexpr float kMatchEpsilon = 0.01f;

    const D3DMATRIX& view = mgr.GetCurrentViewMatrix();
    bool t_view = false;
    const float d_view = BestMatchMatrix4x4(constant_data, view, &t_view);

    float d_view_proj = 1e9f;
    bool t_view_proj = false;
    D3DMATRIX view_proj{};
    if (mgr.HasCurrentProjMatrix()) {
      view_proj = MultiplyD3dMatrices(view, mgr.GetCurrentProjMatrix());
      d_view_proj = BestMatchMatrix4x4(constant_data, view_proj, &t_view_proj);
    }

    float d_world_view_proj = 1e9f;
    bool t_world_view_proj = false;
    D3DMATRIX world_view_proj{};
    if (mgr.HasCurrentWorldMatrix() && mgr.HasCurrentProjMatrix()) {
      const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), view);
      world_view_proj = MultiplyD3dMatrices(world_view, mgr.GetCurrentProjMatrix());
      d_world_view_proj = BestMatchMatrix4x4(constant_data, world_view_proj, &t_world_view_proj);
    }

    if (d_view < kMatchEpsilon) {
      mgr.NoteViewVsConstant(start_register, t_view, constant_data);
    }
    if (d_view_proj < kMatchEpsilon) {
      mgr.NoteViewProjVsConstant(start_register, t_view_proj, constant_data);
    }

    float best = 1e9f;
    RenderHookManager::VsMatrixKind best_kind = RenderHookManager::VsMatrixKind::kNone;
    bool best_t = false;

    if (d_view < best) {
      best = d_view;
      best_kind = RenderHookManager::VsMatrixKind::kView;
      best_t = t_view;
    }
    if (d_view_proj < best) {
      best = d_view_proj;
      best_kind = RenderHookManager::VsMatrixKind::kViewProj;
      best_t = t_view_proj;
    }
    if (d_world_view_proj < best) {
      best = d_world_view_proj;
      best_kind = RenderHookManager::VsMatrixKind::kWorldViewProj;
      best_t = t_world_view_proj;
    }

    if (best < kMatchEpsilon && best_kind != RenderHookManager::VsMatrixKind::kNone) {
      if (mgr.IsInMainRenderPass()) {
        IDirect3DVertexShader9* vs = nullptr;
        if (SUCCEEDED(device->GetVertexShader(&vs)) && vs) {
          mgr.NotePositionVsConstantForShader(vs, best_kind, start_register, best_t, best);
          vs->Release();
        }
      }
    }
  }

  return RenderHookManager::original_set_vertex_shader_constant_f_(
      device, start_register, constant_data, vector4f_count);
}

static HRESULT STDMETHODCALLTYPE DrawIndexedPrimitiveHook(IDirect3DDevice9* device,
                                                         D3DPRIMITIVETYPE type,
                                                         INT base_vertex_index,
                                                         UINT min_vertex_index,
                                                         UINT num_vertices,
                                                         UINT start_index,
                                                         UINT prim_count) {
  RenderHookManager& mgr = RenderHookManager::GetInstance();

  if (!mgr.IsSinglePassStereoEnabled() || !mgr.IsInMainRenderPass() ||
      !mgr.GetLeftEyeRt() || !mgr.GetRightEyeRt()) {
    return RenderHookManager::original_draw_indexed_primitive_(
        device, type, base_vertex_index, min_vertex_index, num_vertices, start_index, prim_count);
  }

  IDirect3DSurface9* cur_rt0 = nullptr;
  if (FAILED(device->GetRenderTarget(0, &cur_rt0)) || !cur_rt0) {
    return RenderHookManager::original_draw_indexed_primitive_(
        device, type, base_vertex_index, min_vertex_index, num_vertices, start_index, prim_count);
  }
  const bool is_main = mgr.IsMainPassSurface(cur_rt0);
  cur_rt0->Release();
  if (!is_main) {
    return RenderHookManager::original_draw_indexed_primitive_(
        device, type, base_vertex_index, min_vertex_index, num_vertices, start_index, prim_count);
  }

  // Save state (including MRTs).
  IDirect3DSurface9* orig_rt0 = nullptr;
  IDirect3DSurface9* orig_rt1 = nullptr;
  IDirect3DSurface9* orig_rt2 = nullptr;
  IDirect3DSurface9* orig_rt3 = nullptr;
  IDirect3DSurface9* orig_depth = nullptr;
  D3DVIEWPORT9 orig_viewport{};

  device->GetRenderTarget(0, &orig_rt0);
  device->GetRenderTarget(1, &orig_rt1);
  device->GetRenderTarget(2, &orig_rt2);
  device->GetRenderTarget(3, &orig_rt3);
  device->GetDepthStencilSurface(&orig_depth);
  device->GetViewport(&orig_viewport);

  D3DVIEWPORT9 vr_viewport{};
  vr_viewport.X = 0;
  vr_viewport.Y = 0;
  vr_viewport.Width = mgr.GetStereoWidth();
  vr_viewport.Height = mgr.GetStereoHeight();
  vr_viewport.MinZ = orig_viewport.MinZ;
  vr_viewport.MaxZ = orig_viewport.MaxZ;

  const D3DMATRIX view_matrix = mgr.GetCurrentViewMatrix();

  // LEFT EYE
  device->SetRenderTarget(0, mgr.GetLeftEyeRt());
  device->SetDepthStencilSurface(mgr.GetLeftEyeDepth());
  device->SetViewport(&vr_viewport);

  D3DMATRIX left_view = view_matrix;
  // Only apply HMD pose here if CameraManager is NOT handling it at scene graph level.
  // Otherwise we get double-application of head position (especially height).
  if (!CameraManager::GetInstance().IsInitialized()) {
    ApplyHmdPoseToViewMatrix(&left_view);
  }
  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kLeft);
  mgr.ApplyEyeOffsetToViewMatrix(&left_view);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &left_view);

  float orig_vs_pos[16];
  RenderHookManager::VsMatrixKind pos_kind = RenderHookManager::VsMatrixKind::kNone;
  UINT pos_reg = 0xFFFFFFFFu;
  bool pos_transpose = false;
  bool can_restore_pos = false;

  if (mgr.IsShaderConstantStereoEnabled() && RenderHookManager::original_set_vertex_shader_constant_f_) {
    can_restore_pos = mgr.GetPositionVsConstantForCurrentShader(device, pos_kind, pos_reg, pos_transpose);
  }
  if (can_restore_pos) {
    can_restore_pos = SUCCEEDED(device->GetVertexShaderConstantF(pos_reg, orig_vs_pos, 4));
  }

  if (can_restore_pos) {
    D3DMATRIX pos_m{};
    if (pos_kind == RenderHookManager::VsMatrixKind::kView) {
      pos_m = left_view;
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kViewProj) {
      D3DMATRIX proj{};
      if (mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kLeft, proj)) {
        pos_m = MultiplyD3dMatrices(left_view, proj);
      } else if (mgr.HasCurrentProjMatrix()) {
        pos_m = MultiplyD3dMatrices(left_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = left_view;
      }
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kWorldViewProj) {
      D3DMATRIX proj{};
      if (mgr.HasCurrentWorldMatrix() && mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kLeft, proj)) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), left_view);
        pos_m = MultiplyD3dMatrices(world_view, proj);
      } else if (mgr.HasCurrentWorldMatrix() && mgr.HasCurrentProjMatrix()) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), left_view);
        pos_m = MultiplyD3dMatrices(world_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = left_view;
      }
    } else {
      pos_m = left_view;
    }

    float left_vs_pos[16];
    CopyD3dMatrixToFloat16(pos_m, left_vs_pos, pos_transpose);
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, left_vs_pos, 4);
  }

  const HRESULT hr = RenderHookManager::original_draw_indexed_primitive_(
      device, type, base_vertex_index, min_vertex_index, num_vertices, start_index, prim_count);

  // RIGHT EYE
  device->SetRenderTarget(0, mgr.GetRightEyeRt());
  device->SetDepthStencilSurface(mgr.GetRightEyeDepth());

  D3DMATRIX right_view = view_matrix;
  if (!CameraManager::GetInstance().IsInitialized()) {
    ApplyHmdPoseToViewMatrix(&right_view);
  }
  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kRight);
  mgr.ApplyEyeOffsetToViewMatrix(&right_view);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &right_view);

  if (can_restore_pos) {
    D3DMATRIX pos_m{};
    if (pos_kind == RenderHookManager::VsMatrixKind::kView) {
      pos_m = right_view;
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kViewProj) {
      D3DMATRIX proj{};
      if (mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kRight, proj)) {
        pos_m = MultiplyD3dMatrices(right_view, proj);
      } else if (mgr.HasCurrentProjMatrix()) {
        pos_m = MultiplyD3dMatrices(right_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = right_view;
      }
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kWorldViewProj) {
      D3DMATRIX proj{};
      if (mgr.HasCurrentWorldMatrix() && mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kRight, proj)) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), right_view);
        pos_m = MultiplyD3dMatrices(world_view, proj);
      } else if (mgr.HasCurrentWorldMatrix() && mgr.HasCurrentProjMatrix()) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), right_view);
        pos_m = MultiplyD3dMatrices(world_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = right_view;
      }
    } else {
      pos_m = right_view;
    }

    float right_vs_pos[16];
    CopyD3dMatrixToFloat16(pos_m, right_vs_pos, pos_transpose);
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, right_vs_pos, 4);
  }

  RenderHookManager::original_draw_indexed_primitive_(
      device, type, base_vertex_index, min_vertex_index, num_vertices, start_index, prim_count);

  if (can_restore_pos) {
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, orig_vs_pos, 4);
  }

  // Restore original state.
  device->SetRenderTarget(0, orig_rt0);
  device->SetRenderTarget(1, orig_rt1);
  device->SetRenderTarget(2, orig_rt2);
  device->SetRenderTarget(3, orig_rt3);
  device->SetDepthStencilSurface(orig_depth);
  device->SetViewport(&orig_viewport);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &view_matrix);

  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kNone);

  if (orig_rt0) orig_rt0->Release();
  if (orig_rt1) orig_rt1->Release();
  if (orig_rt2) orig_rt2->Release();
  if (orig_rt3) orig_rt3->Release();
  if (orig_depth) orig_depth->Release();

  return hr;
}

static HRESULT STDMETHODCALLTYPE DrawPrimitiveHook(IDirect3DDevice9* device,
                                                  D3DPRIMITIVETYPE primitive_type,
                                                  UINT start_vertex,
                                                  UINT primitive_count) {
  RenderHookManager& mgr = RenderHookManager::GetInstance();

  if (!mgr.IsSinglePassStereoEnabled() || !mgr.IsInMainRenderPass() ||
      !mgr.GetLeftEyeRt() || !mgr.GetRightEyeRt()) {
    return RenderHookManager::original_draw_primitive_(device, primitive_type, start_vertex, primitive_count);
  }

  IDirect3DSurface9* cur_rt0 = nullptr;
  if (FAILED(device->GetRenderTarget(0, &cur_rt0)) || !cur_rt0) {
    return RenderHookManager::original_draw_primitive_(device, primitive_type, start_vertex, primitive_count);
  }
  const bool is_main = mgr.IsMainPassSurface(cur_rt0);
  cur_rt0->Release();
  if (!is_main) {
    return RenderHookManager::original_draw_primitive_(device, primitive_type, start_vertex, primitive_count);
  }

  IDirect3DSurface9* orig_rt0 = nullptr;
  IDirect3DSurface9* orig_depth = nullptr;
  D3DVIEWPORT9 orig_viewport{};
  device->GetRenderTarget(0, &orig_rt0);
  device->GetDepthStencilSurface(&orig_depth);
  device->GetViewport(&orig_viewport);

  D3DVIEWPORT9 vr_viewport{};
  vr_viewport.X = 0;
  vr_viewport.Y = 0;
  vr_viewport.Width = mgr.GetStereoWidth();
  vr_viewport.Height = mgr.GetStereoHeight();
  vr_viewport.MinZ = orig_viewport.MinZ;
  vr_viewport.MaxZ = orig_viewport.MaxZ;

  const D3DMATRIX view_matrix = mgr.GetCurrentViewMatrix();

  device->SetRenderTarget(0, mgr.GetLeftEyeRt());
  device->SetDepthStencilSurface(mgr.GetLeftEyeDepth());
  device->SetViewport(&vr_viewport);

  D3DMATRIX left_view = view_matrix;
  if (!CameraManager::GetInstance().IsInitialized()) {
    ApplyHmdPoseToViewMatrix(&left_view);
  }
  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kLeft);
  mgr.ApplyEyeOffsetToViewMatrix(&left_view);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &left_view);

  float orig_vs_pos[16];
  RenderHookManager::VsMatrixKind pos_kind = RenderHookManager::VsMatrixKind::kNone;
  UINT pos_reg = 0xFFFFFFFFu;
  bool pos_transpose = false;
  bool can_restore_pos = false;

  if (mgr.IsShaderConstantStereoEnabled() && RenderHookManager::original_set_vertex_shader_constant_f_) {
    can_restore_pos = mgr.GetPositionVsConstantForCurrentShader(device, pos_kind, pos_reg, pos_transpose);
  }
  if (can_restore_pos) {
    can_restore_pos = SUCCEEDED(device->GetVertexShaderConstantF(pos_reg, orig_vs_pos, 4));
  }

  if (can_restore_pos) {
    D3DMATRIX pos_m{};
    if (pos_kind == RenderHookManager::VsMatrixKind::kView) {
      pos_m = left_view;
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kViewProj) {
      D3DMATRIX proj{};
      if (mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kLeft, proj)) {
        pos_m = MultiplyD3dMatrices(left_view, proj);
      } else if (mgr.HasCurrentProjMatrix()) {
        pos_m = MultiplyD3dMatrices(left_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = left_view;
      }
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kWorldViewProj) {
      D3DMATRIX proj{};
      if (mgr.HasCurrentWorldMatrix() && mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kLeft, proj)) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), left_view);
        pos_m = MultiplyD3dMatrices(world_view, proj);
      } else if (mgr.HasCurrentWorldMatrix() && mgr.HasCurrentProjMatrix()) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), left_view);
        pos_m = MultiplyD3dMatrices(world_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = left_view;
      }
    } else {
      pos_m = left_view;
    }

    float left_vs_pos[16];
    CopyD3dMatrixToFloat16(pos_m, left_vs_pos, pos_transpose);
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, left_vs_pos, 4);
  }

  const HRESULT hr = RenderHookManager::original_draw_primitive_(device, primitive_type, start_vertex, primitive_count);

  device->SetRenderTarget(0, mgr.GetRightEyeRt());
  device->SetDepthStencilSurface(mgr.GetRightEyeDepth());

  D3DMATRIX right_view = view_matrix;
  if (!CameraManager::GetInstance().IsInitialized()) {
    ApplyHmdPoseToViewMatrix(&right_view);
  }
  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kRight);
  mgr.ApplyEyeOffsetToViewMatrix(&right_view);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &right_view);

  if (can_restore_pos) {
    D3DMATRIX pos_m{};
    if (pos_kind == RenderHookManager::VsMatrixKind::kView) {
      pos_m = right_view;
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kViewProj) {
      D3DMATRIX proj{};
      if (mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kRight, proj)) {
        pos_m = MultiplyD3dMatrices(right_view, proj);
      } else if (mgr.HasCurrentProjMatrix()) {
        pos_m = MultiplyD3dMatrices(right_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = right_view;
      }
    } else if (pos_kind == RenderHookManager::VsMatrixKind::kWorldViewProj) {
      D3DMATRIX proj{};
      if (mgr.HasCurrentWorldMatrix() && mgr.GetEyeProjectionMatrix(RenderHookManager::EyePass::kRight, proj)) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), right_view);
        pos_m = MultiplyD3dMatrices(world_view, proj);
      } else if (mgr.HasCurrentWorldMatrix() && mgr.HasCurrentProjMatrix()) {
        const D3DMATRIX world_view = MultiplyD3dMatrices(mgr.GetCurrentWorldMatrix(), right_view);
        pos_m = MultiplyD3dMatrices(world_view, mgr.GetCurrentProjMatrix());
      } else {
        pos_m = right_view;
      }
    } else {
      pos_m = right_view;
    }

    float right_vs_pos[16];
    CopyD3dMatrixToFloat16(pos_m, right_vs_pos, pos_transpose);
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, right_vs_pos, 4);
  }

  RenderHookManager::original_draw_primitive_(device, primitive_type, start_vertex, primitive_count);

  if (can_restore_pos) {
    RenderHookManager::original_set_vertex_shader_constant_f_(device, pos_reg, orig_vs_pos, 4);
  }

  device->SetRenderTarget(0, orig_rt0);
  device->SetDepthStencilSurface(orig_depth);
  device->SetViewport(&orig_viewport);
  RenderHookManager::original_set_transform_(device, D3DTS_VIEW, &view_matrix);

  mgr.SetCurrentEyePass(RenderHookManager::EyePass::kNone);

  if (orig_rt0) orig_rt0->Release();
  if (orig_depth) orig_depth->Release();

  return hr;
}

}  // namespace

// Static member definitions.
RenderHookManager::RenderFunc RenderHookManager::original_render_ = nullptr;
RenderHookManager::ProcessImageSpaceShadersFunc RenderHookManager::original_process_image_space_shaders_ = nullptr;
RenderHookManager::CreateBackgroundTextureFunc RenderHookManager::original_create_background_texture_ = nullptr;
RenderHookManager::CreateBackgroundTextureFunc RenderHookManager::original_pipboy_render_ = nullptr;
RenderHookManager::CreateBackgroundTextureFunc RenderHookManager::original_3d_menu_render_ = nullptr;
RenderHookManager::SetTransformFunc RenderHookManager::original_set_transform_ = nullptr;
RenderHookManager::DrawIndexedPrimitiveFunc RenderHookManager::original_draw_indexed_primitive_ = nullptr;
RenderHookManager::DrawPrimitiveFunc RenderHookManager::original_draw_primitive_ = nullptr;
RenderHookManager::SetVertexShaderConstantFFunc RenderHookManager::original_set_vertex_shader_constant_f_ = nullptr;

RenderHookManager& RenderHookManager::GetInstance() {
  static RenderHookManager instance;
  return instance;
}

void RenderHookManager::Initialize() {
  _MESSAGE("Initializing RenderHookManager with Microsoft Detours...");

  constexpr uint32_t kMainRenderAddr = 0x008706B0;

  original_render_ = reinterpret_cast<RenderFunc>(kMainRenderAddr);

  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(reinterpret_cast<PVOID*>(&original_render_), reinterpret_cast<PVOID>(MainRenderHook));
  const LONG err = DetourTransactionCommit();

  if (err == NO_ERROR) {
    _MESSAGE("Main::Render hook installed successfully using Detours!");
  } else {
    _ERROR("Failed to install Main::Render hook! Detours error: %d", err);
  }

  // Optional: UI/post-process hooks removed for now.
}

void RenderHookManager::Update() {
  // No deferred installation; hooks attach immediately.
}

void RenderHookManager::Shutdown() {
  // No explicit detach on shutdown for now.
  // Fallout is shutting down; leaving hooks is acceptable.
}

void RenderHookManager::UpdateEyeData() {
  VrManager& vr = VrManager::GetInstance();

  vr::HmdMatrix34_t left_eye_to_head{};
  vr::HmdMatrix34_t right_eye_to_head{};
  vr.GetEyeViews(left_eye_to_head, right_eye_to_head);

  left_eye_data_.eye_to_head = left_eye_to_head;
  left_eye_data_.projection = vr.GetProjectionMatrix(vr::Eye_Left, 1.0f, 250000.0f);
  left_eye_data_.is_valid = true;

  right_eye_data_.eye_to_head = right_eye_to_head;
  right_eye_data_.projection = vr.GetProjectionMatrix(vr::Eye_Right, 1.0f, 250000.0f);
  right_eye_data_.is_valid = true;

  if (left_eye_data_.is_valid && right_eye_data_.is_valid) {
    CalculateProjectionMatrix(left_eye_data_.projection, reinterpret_cast<float*>(&left_eye_proj_matrix_));
    CalculateProjectionMatrix(right_eye_data_.projection, reinterpret_cast<float*>(&right_eye_proj_matrix_));
    has_eye_proj_matrices_ = true;
  }
}

void RenderHookManager::CalculateProjectionMatrix(const vr::HmdMatrix44_t& vr_proj, float* out_matrix) {
  if (!out_matrix) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      out_matrix[i * 4 + j] = vr_proj.m[i][j];
    }
  }

  // Preserve original behavior: widen FOV slightly.
  constexpr float kFovScale = 1.15f;
  out_matrix[0] /= kFovScale;
  out_matrix[5] /= kFovScale;
}

bool RenderHookManager::GetEyeProjectionMatrix(EyePass eye, D3DMATRIX& out_matrix) const {
  if (!has_eye_proj_matrices_) {
    return false;
  }
  if (eye == EyePass::kLeft) {
    out_matrix = left_eye_proj_matrix_;
    return true;
  }
  if (eye == EyePass::kRight) {
    out_matrix = right_eye_proj_matrix_;
    return true;
  }
  return false;
}

void RenderHookManager::ApplyEyeOffsetToViewMatrix(D3DMATRIX* view_matrix) {
  if (!view_matrix) {
    return;
  }

  const EyePass eye = current_eye_pass_;
  if (eye == EyePass::kNone) {
    return;
  }

  VrManager& vr = VrManager::GetInstance();
  const vr::HmdMatrix34_t eye_to_head = vr.GetEyeToHeadTransform(
      (eye == EyePass::kLeft) ? vr::Eye_Left : vr::Eye_Right);

  const float eye_offset_meters = eye_to_head.m[0][3];
  constexpr float kUnitsPerMeter = 69.0f;
  const float eye_offset_units = eye_offset_meters * kUnitsPerMeter;

  const float right_x = view_matrix->m[0][0];
  const float right_y = view_matrix->m[0][1];
  const float right_z = view_matrix->m[0][2];

  view_matrix->m[3][0] -= eye_offset_units * right_x;
  view_matrix->m[3][1] -= eye_offset_units * right_y;
  view_matrix->m[3][2] -= eye_offset_units * right_z;
}

void RenderHookManager::HookD3d9SetTransform(IDirect3DDevice9* device) {
  if (set_transform_hooked_ || !device) {
    return;
  }

  void** const vtable = *reinterpret_cast<void***>(device);
  constexpr int kSetTransformVtableIndex = 44;

  original_set_transform_ = reinterpret_cast<SetTransformFunc>(vtable[kSetTransformVtableIndex]);
  _MESSAGE("Hooking D3D9 SetTransform at vtable[%d] = %p", kSetTransformVtableIndex,
           vtable[kSetTransformVtableIndex]);

  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(reinterpret_cast<PVOID*>(&original_set_transform_),
               reinterpret_cast<PVOID>(SetTransformHook));
  const LONG result = DetourTransactionCommit();

  if (result == NO_ERROR) {
    _MESSAGE("D3D9 SetTransform hook installed successfully!");
    set_transform_hooked_ = true;
    d3d9_device_ = device;

    HookD3d9DrawCalls(device);
    HookD3d9ShaderConstants(device);

    VrManager::GetInstance().EnsureEyeTexturesCreated();
  } else {
    _ERROR("Failed to hook D3D9 SetTransform: error %d", result);
  }
}

void RenderHookManager::HookD3d9DrawCalls(IDirect3DDevice9* device) {
  if (draw_calls_hooked_ || !device) {
    return;
  }

  void** const vtable = *reinterpret_cast<void***>(device);
  constexpr int kDrawPrimitiveVtableIndex = 81;
  constexpr int kDrawIndexedPrimitiveVtableIndex = 82;

  original_draw_primitive_ = reinterpret_cast<DrawPrimitiveFunc>(vtable[kDrawPrimitiveVtableIndex]);
  original_draw_indexed_primitive_ =
      reinterpret_cast<DrawIndexedPrimitiveFunc>(vtable[kDrawIndexedPrimitiveVtableIndex]);

  _MESSAGE("Hooking D3D9 DrawPrimitive at vtable[%d] = %p", kDrawPrimitiveVtableIndex,
           vtable[kDrawPrimitiveVtableIndex]);
  _MESSAGE("Hooking D3D9 DrawIndexedPrimitive at vtable[%d] = %p", kDrawIndexedPrimitiveVtableIndex,
           vtable[kDrawIndexedPrimitiveVtableIndex]);

  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(reinterpret_cast<PVOID*>(&original_draw_primitive_),
               reinterpret_cast<PVOID>(DrawPrimitiveHook));
  DetourAttach(reinterpret_cast<PVOID*>(&original_draw_indexed_primitive_),
               reinterpret_cast<PVOID>(DrawIndexedPrimitiveHook));
  const LONG result = DetourTransactionCommit();

  if (result == NO_ERROR) {
    _MESSAGE("D3D9 draw call hooks installed successfully for single-pass stereo!");
    draw_calls_hooked_ = true;
  } else {
    _ERROR("Failed to hook D3D9 draw calls: error %d", result);
  }
}

void RenderHookManager::HookD3d9ShaderConstants(IDirect3DDevice9* device) {
  if (shader_constants_hooked_ || !device) {
    return;
  }

  if (!shader_constant_stereo_enabled_) {
    _MESSAGE("Shader constant stereo disabled; skipping SetVertexShaderConstantF hook");
    shader_constants_hooked_ = true;
    return;
  }

  void** const vtable = *reinterpret_cast<void***>(device);
  constexpr int kSetVsConstFVtableIndex = 94;
  void* const p_set_vs_const_f = vtable[kSetVsConstFVtableIndex];

  HMODULE d3d9_module = GetModuleHandleA("d3d9.dll");
  MODULEINFO mod_info{};
  if (!d3d9_module ||
      !GetModuleInformation(GetCurrentProcess(), d3d9_module, &mod_info, sizeof(mod_info))) {
    _MESSAGE("Warning: could not query d3d9.dll module info; skipping VS constant hook");
    return;
  }

  const uintptr_t mod_start = reinterpret_cast<uintptr_t>(mod_info.lpBaseOfDll);
  const uintptr_t mod_end = mod_start + mod_info.SizeOfImage;
  const uintptr_t fn_addr = reinterpret_cast<uintptr_t>(p_set_vs_const_f);
  if (fn_addr < mod_start || fn_addr >= mod_end) {
    _MESSAGE("Warning: vtable[%d] not in d3d9.dll (%p); skipping VS constant hook",
             kSetVsConstFVtableIndex, p_set_vs_const_f);
    return;
  }

  original_set_vertex_shader_constant_f_ =
      reinterpret_cast<SetVertexShaderConstantFFunc>(p_set_vs_const_f);

  _MESSAGE("Hooking D3D9 SetVertexShaderConstantF at vtable[%d] = %p", kSetVsConstFVtableIndex,
           p_set_vs_const_f);

  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(reinterpret_cast<PVOID*>(&original_set_vertex_shader_constant_f_),
               reinterpret_cast<PVOID>(SetVertexShaderConstantFHook));
  const LONG result = DetourTransactionCommit();

  if (result == NO_ERROR) {
    _MESSAGE("D3D9 SetVertexShaderConstantF hook installed successfully!");
    shader_constants_hooked_ = true;
  } else {
    _ERROR("Failed to hook D3D9 SetVertexShaderConstantF: error %d", result);
  }
}

IDirect3DSurface9* RenderHookManager::GetLeftEyeRt() const {
  return VrManager::GetInstance().GetEyeRenderTarget(0);
}

IDirect3DSurface9* RenderHookManager::GetRightEyeRt() const {
  return VrManager::GetInstance().GetEyeRenderTarget(1);
}

IDirect3DSurface9* RenderHookManager::GetLeftEyeDepth() const {
  return VrManager::GetInstance().GetEyeDepthStencil(0);
}

IDirect3DSurface9* RenderHookManager::GetRightEyeDepth() const {
  return VrManager::GetInstance().GetEyeDepthStencil(1);
}

uint32_t RenderHookManager::GetStereoWidth() const {
  return VrManager::GetInstance().GetEyeTextureWidth();
}

uint32_t RenderHookManager::GetStereoHeight() const {
  return VrManager::GetInstance().GetEyeTextureHeight();
}

void RenderHookManager::SetMainPassSurfaces(IDirect3DSurface9* rt0,
                                           IDirect3DSurface9* depth,
                                           const D3DVIEWPORT9& viewport) {
  if (main_pass_rt0_) {
    main_pass_rt0_->Release();
    main_pass_rt0_ = nullptr;
  }
  if (main_pass_depth_) {
    main_pass_depth_->Release();
    main_pass_depth_ = nullptr;
  }

  if (rt0) {
    rt0->AddRef();
    main_pass_rt0_ = rt0;
  }
  if (depth) {
    depth->AddRef();
    main_pass_depth_ = depth;
  }

  main_pass_viewport_ = viewport;
}

bool RenderHookManager::IsMainPassSurface(IDirect3DSurface9* rt0) const {
  return rt0 && main_pass_rt0_ && (rt0 == main_pass_rt0_);
}

void RenderHookManager::NoteViewVsConstant(uint32_t start_register,
                                          bool transposed,
                                          const float* matrix16) {
  if (!matrix16) {
    return;
  }
  view_vs_const_reg_ = start_register;
  view_vs_const_transposed_ = transposed;
  last_view_vs_const_valid_ = true;
  memcpy(last_view_vs_const_, matrix16, sizeof(float) * 16);
}

void RenderHookManager::NoteViewProjVsConstant(uint32_t start_register,
                                              bool transposed,
                                              const float* matrix16) {
  if (!matrix16) {
    return;
  }
  view_proj_vs_const_reg_ = start_register;
  view_proj_vs_const_transposed_ = transposed;
  last_view_proj_vs_const_valid_ = true;
  memcpy(last_view_proj_vs_const_, matrix16, sizeof(float) * 16);
}

void RenderHookManager::NotePositionVsConstant(VsMatrixKind kind,
                                              uint32_t start_register,
                                              bool transposed,
                                              const float* matrix16,
                                              float diff) {
  if (!matrix16 || kind == VsMatrixKind::kNone) {
    return;
  }

  if (!last_pos_vs_const_valid_ || diff < (pos_vs_best_diff_ - 1e-6f)) {
    pos_vs_best_diff_ = diff;
    pos_vs_kind_ = kind;
    pos_vs_const_reg_ = start_register;
    pos_vs_const_transposed_ = transposed;
    last_pos_vs_const_valid_ = true;
    memcpy(last_pos_vs_const_, matrix16, sizeof(float) * 16);
  }
}

void RenderHookManager::NotePositionVsConstantForShader(IDirect3DVertexShader9* vs,
                                                       VsMatrixKind kind,
                                                       uint32_t start_register,
                                                       bool transposed,
                                                       float diff) {
  if (!vs || kind == VsMatrixKind::kNone) {
    return;
  }

  ShaderPosConstInfo& info = shader_pos_consts_[vs];
  if (info.frame_id != frame_id_ || diff < (info.diff - 1e-6f)) {
    info.kind = kind;
    info.reg = start_register;
    info.transposed = transposed;
    info.diff = diff;
    info.frame_id = frame_id_;
  }
}

bool RenderHookManager::GetPositionVsConstantForCurrentShader(IDirect3DDevice9* device,
                                                             VsMatrixKind& out_kind,
                                                             uint32_t& out_reg,
                                                             bool& out_transposed) const {
  if (!device) {
    return false;
  }

  IDirect3DVertexShader9* vs = nullptr;
  if (FAILED(device->GetVertexShader(&vs)) || !vs) {
    return false;
  }

  const auto it = shader_pos_consts_.find(vs);
  vs->Release();

  if (it == shader_pos_consts_.end()) {
    return false;
  }

  out_kind = it->second.kind;
  out_reg = it->second.reg;
  out_transposed = it->second.transposed;
  return true;
}

void RenderHookManager::OnNewFrame() {
  eye_marker_drawn_left_ = false;
  eye_marker_drawn_right_ = false;
  ++frame_id_;
}

void RenderHookManager::MaybeDrawEyeMarker(IDirect3DDevice9* device, EyePass eye) {
  if (!eye_marker_enabled_ || !device) {
    return;
  }

  if (eye == EyePass::kLeft) {
    if (eye_marker_drawn_left_) {
      return;
    }
    eye_marker_drawn_left_ = true;
  } else if (eye == EyePass::kRight) {
    if (eye_marker_drawn_right_) {
      return;
    }
    eye_marker_drawn_right_ = true;
  } else {
    return;
  }

  const uint32_t w = GetStereoWidth();
  const uint32_t h = GetStereoHeight();
  const LONG size = static_cast<LONG>((eye_marker_size_px_ < 64) ? 256 : eye_marker_size_px_);
  const LONG cx = static_cast<LONG>(w / 2);
  const LONG cy = static_cast<LONG>(h / 2);

  RECT r{};
  r.left = cx - (size / 2);
  r.top = cy - (size / 2);
  r.right = r.left + size;
  r.bottom = r.top + size;

  const D3DCOLOR color = (eye == EyePass::kLeft) ? kEyeMarkerLeftColor : kEyeMarkerRightColor;
  IDirect3DSurface9* surf = (eye == EyePass::kLeft) ? GetLeftEyeRt() : GetRightEyeRt();
  if (surf) {
    device->ColorFill(surf, &r, color);
  }
}

void RenderHookManager::ForceStampEyeMarkers(IDirect3DDevice9* device) {
  if (!eye_marker_enabled_ || !device) {
    return;
  }

  eye_marker_drawn_left_ = false;
  eye_marker_drawn_right_ = false;

  const uint32_t w = GetStereoWidth();
  const uint32_t h = GetStereoHeight();
  const LONG size = 256;
  const LONG cx = static_cast<LONG>(w / 2);
  const LONG cy = static_cast<LONG>(h / 2);

  RECT r_left{cx - (size / 2), cy - (size / 2), cx + (size / 2), cy + (size / 2)};
  RECT r_right = r_left;

  if (IDirect3DSurface9* surf = GetLeftEyeRt()) {
    device->ColorFill(surf, &r_left, kEyeMarkerLeftColor);
  }
  if (IDirect3DSurface9* surf = GetRightEyeRt()) {
    device->ColorFill(surf, &r_right, kEyeMarkerRightColor);
  }
}

void RenderHookManager::ClearDepthBuffer() {
  if (!kGameD3d9DevicePtr || !*kGameD3d9DevicePtr) {
    return;
  }

  IDirect3DDevice9* device = *kGameD3d9DevicePtr;
  device->Clear(0, nullptr, D3DCLEAR_ZBUFFER | D3DCLEAR_STENCIL, 0, 1.0f, 0);
}

void RenderHookManager::ClearRenderTarget() {
  if (!kGameD3d9DevicePtr || !*kGameD3d9DevicePtr) {
    return;
  }

  IDirect3DDevice9* device = *kGameD3d9DevicePtr;
  device->Clear(0, nullptr,
                D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER | D3DCLEAR_STENCIL,
                D3DCOLOR_ARGB(0, 0, 0, 0), 1.0f, 0);
}

void RenderHookManager::CaptureCurrentEye(int eye) {
  VrManager::GetInstance().MarkEyeCaptured(eye);
}

}  // namespace nvse_vr
