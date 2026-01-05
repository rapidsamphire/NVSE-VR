#pragma once

#include <Windows.h>

// Minimal assertion support for NVSE/common consumers.
// NVSE headers frequently use ASSERT() in inline helpers.

__declspec(noreturn) inline void _AssertionFailed(const char* file, unsigned long line, const char* desc)
{
  char buffer[1024] = {0};
  wsprintfA(buffer, "Assertion failed in %s (%lu): %s\n", file, line, desc);
  OutputDebugStringA(buffer);
  __debugbreak();
  TerminateProcess(GetCurrentProcess(), 0xDEAD);
}

#define ASSERT(expr) do { if(!(expr)) { _AssertionFailed(__FILE__, __LINE__, #expr); } } while(0)
