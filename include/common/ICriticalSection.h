#pragma once

#include <Windows.h>

class ICriticalSection
{
public:
  ICriticalSection() { InitializeCriticalSection(&critSection); }
  ~ICriticalSection() { DeleteCriticalSection(&critSection); }

  void Enter() { EnterCriticalSection(&critSection); }
  void Leave() { LeaveCriticalSection(&critSection); }
  bool TryEnter() { return TryEnterCriticalSection(&critSection) != 0; }

private:
  CRITICAL_SECTION critSection;
};
