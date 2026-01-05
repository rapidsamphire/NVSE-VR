#pragma once

#include <cstdint>
#include <cstring>

// NVSE-style primitive typedefs.
// Use Windows-sized base types to match NVSE headers' expectations on x86.

typedef unsigned char UInt8;
typedef unsigned short UInt16;
typedef unsigned long UInt32;
typedef unsigned long long UInt64;
typedef signed char SInt8;
typedef signed short SInt16;
typedef signed long SInt32;
typedef signed long long SInt64;
typedef float Float32;
typedef double Float64;

// Simple Bitfield helper used throughout NVSE headers.
template <class T>
class Bitfield
{
public:
  Bitfield() : data(0) {}
  explicit Bitfield(T value) : data(value) {}

  bool IsSet(T mask) const { return (data & mask) != 0; }
  void Set(T mask) { data = static_cast<T>(data | mask); }
  void Clear(T mask) { data = static_cast<T>(data & ~mask); }
  void Write(T mask, bool value) { value ? Set(mask) : Clear(mask); }

  operator T() const { return data; }

private:
  T data;
};

typedef Bitfield<UInt8> Bitfield8;
typedef Bitfield<UInt16> Bitfield16;
typedef Bitfield<UInt32> Bitfield32;

#define CHAR_CODE(a, b, c, d) (((a & 0xFF) << 0) | ((b & 0xFF) << 8) | ((c & 0xFF) << 16) | ((d & 0xFF) << 24))
