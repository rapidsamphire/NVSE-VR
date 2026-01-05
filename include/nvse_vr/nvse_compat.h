#pragma once

// NVSE headers in this workspace rely on typedefs and STL headers that are
// normally provided via the NVSE solution's PCH/prefix headers.
// The CMake plugin build does not use those PCHs, so we provide the minimal
// compatibility surface here.

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>

// Prefer NVSE/common-style types + logging macros if available via include paths.
// In this workspace, the needed `common/` headers are provided by the Stewie Tweaks source.
#if defined(__has_include) && __has_include("common/ITypes.h")
#define NVSE_VR_HAS_NVSE_COMMON 1
#include "common/IErrors.h"
#include "common/ITypes.h"
#include "common/IDebugLog.h"  // defines _MESSAGE/_ERROR and declares `extern IDebugLog gLog`
#else
#define NVSE_VR_HAS_NVSE_COMMON 0
using UInt8 = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;

using SInt8 = std::int8_t;
using SInt16 = std::int16_t;
using SInt32 = std::int32_t;
using SInt64 = std::int64_t;

// Minimal logging fallbacks (prefer `common/IDebugLog.h` when present).
#ifndef _MESSAGE
#define _MESSAGE(...) ((void)0)
#endif
#ifndef _ERROR
#define _ERROR(...) ((void)0)
#endif
#endif

// NVSE uses these helpers throughout its headers.
template <class K, class V>
using UnorderedMap = std::unordered_map<K, V>;

// A large portion of the NVSE headers rely on a STATIC_ASSERT macro.
// Force our definition to avoid collisions with any Windows/SDK macros.
#ifdef STATIC_ASSERT
#undef STATIC_ASSERT
#endif
#define STATIC_ASSERT(expr) static_assert((expr), #expr)

// NVSE headers sometimes assume ASSERT() exists.
#if !defined(ASSERT)
#define ASSERT(expr) do { if(!(expr)) { __debugbreak(); } } while(0)
#endif
