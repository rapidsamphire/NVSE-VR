#include <Windows.h>

#include "nvse_vr/nvse_compat.h"

#include "nvse/PluginAPI.h"

#include "nvse_vr/camera_manager.h"
#include "nvse_vr/render_hook_manager.h"
#include "nvse_vr/vr_manager.h"

IDebugLog gLog("nvse_plugin_vr.log");
PluginHandle g_plugin_handle = kPluginHandle_Invalid;

static NVSEMessagingInterface* g_messaging = nullptr;

static void MessageHandler(NVSEMessagingInterface::Message* msg) {
  if (!msg) {
    return;
  }

  switch (msg->type) {
    case NVSEMessagingInterface::kMessage_DeferredInit: {
      // Install the main render hook as early as possible.
      nvse_vr::RenderHookManager::GetInstance().Initialize();
      // VrManager will self-initialize when SteamVR/HMD becomes available.
      nvse_vr::VrManager::GetInstance().Initialize();
      // Install camera hooks for VR head tracking.
      nvse_vr::CameraManager::GetInstance().Initialize();
      break;
    }
    case NVSEMessagingInterface::kMessage_MainGameLoop: {
      nvse_vr::VrManager::GetInstance().Update();
      nvse_vr::RenderHookManager::GetInstance().Update();
      break;
    }
    default:
      break;
  }
}

extern "C" {

bool __declspec(dllexport) NVSEPlugin_Query(const NVSEInterface* nvse, PluginInfo* info) {
  if (!nvse || !info) {
    return false;
  }

  _MESSAGE("=== NVSE-VR REWRITE (CMAKE BUILD) ===");
  _MESSAGE("nvse_plugin_vr loaded from rewrite codebase");

  info->infoVersion = PluginInfo::kInfoVersion;
  info->name = "nvse_plugin_vr";
  info->version = 1;

  if (nvse->isEditor) {
    _MESSAGE("nvse_plugin_vr: editor mode not supported");
    return false;
  }

  g_plugin_handle = nvse->GetPluginHandle();

  g_messaging = static_cast<NVSEMessagingInterface*>(nvse->QueryInterface(kInterface_Messaging));
  if (!g_messaging) {
    _ERROR("nvse_plugin_vr: failed to acquire NVSE messaging interface");
    return false;
  }

  return true;
}

bool __declspec(dllexport) NVSEPlugin_Load(NVSEInterface* nvse) {
  if (!nvse || !g_messaging) {
    return false;
  }

  g_messaging->RegisterListener(g_plugin_handle, "NVSE", MessageHandler);

  _MESSAGE("nvse_plugin_vr loaded");
  return true;
}

}  // extern "C"
