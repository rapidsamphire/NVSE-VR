#include <Windows.h>

#include "common/IDebugLog.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

std::FILE* IDebugLog::logFile = nullptr;
bool IDebugLog::autoFlush = true;
IDebugLog::LogLevel IDebugLog::logLevel = IDebugLog::kLevel_DebugMessage;
IDebugLog::LogLevel IDebugLog::printLevel = IDebugLog::kLevel_Message;
int IDebugLog::indentLevel = 0;
char IDebugLog::sourceBuf[16] = {0};

IDebugLog::IDebugLog() { EnsureOpen(); }

IDebugLog::IDebugLog(const char* name)
{
  if (name && name[0]) {
    Open(name);
  } else {
    EnsureOpen();
  }
}

IDebugLog::~IDebugLog() = default;

void IDebugLog::EnsureOpen()
{
  if (logFile) {
    return;
  }

  // Default log path; NVSE plugins write in the Game root folder
  Open("nvse_plugin_vr.log");
}

void IDebugLog::Open(const char* path)
{
  if (logFile) {
    std::fclose(logFile);
    logFile = nullptr;
  }

  FILE* file = nullptr;
  fopen_s(&file, path, "w");
  logFile = file;
}

void IDebugLog::OpenRelative(int /*folderID*/, const char* relPath)
{
  // Minimal: treat as relative to CWD.
  Open(relPath);
}

void IDebugLog::SetSource(const char* source)
{
  if (!source) {
    sourceBuf[0] = 0;
    return;
  }

  std::strncpy(sourceBuf, source, sizeof(sourceBuf) - 1);
  sourceBuf[sizeof(sourceBuf) - 1] = 0;
}

void IDebugLog::ClearSource(void)
{
  sourceBuf[0] = 0;
}

void IDebugLog::Indent(void)
{
  ++indentLevel;
}

void IDebugLog::Outdent(void)
{
  if (indentLevel > 0) {
    --indentLevel;
  }
}

void IDebugLog::OpenBlock(void) {}
void IDebugLog::CloseBlock(void) {}

void IDebugLog::SetAutoFlush(bool inAutoFlush)
{
  autoFlush = inAutoFlush;
}

void IDebugLog::Message(const char* message, const char* source)
{
  EnsureOpen();
  if (!logFile || !message) {
    return;
  }

  if (source) {
    SetSource(source);
  }

  for (int i = 0; i < indentLevel; ++i) {
    std::fputs("\t", logFile);
  }

  if (sourceBuf[0]) {
    std::fprintf(logFile, "[%s] ", sourceBuf);
  }

  std::fputs(message, logFile);
  std::fputs("\n", logFile);

  if (autoFlush) {
    std::fflush(logFile);
  }
}

void IDebugLog::FormattedMessage(const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  FormattedMessage(fmt, args);
  va_end(args);
}

void IDebugLog::FormattedMessage(const char* fmt, va_list args)
{
  char buffer[8192];
  buffer[0] = 0;
  vsnprintf_s(buffer, sizeof(buffer), _TRUNCATE, fmt, args);
  Message(buffer);
}

void IDebugLog::Log(LogLevel level, const char* fmt, va_list args)
{
  if (level > logLevel && level > printLevel) {
    return;
  }

  char buffer[8192];
  buffer[0] = 0;
  vsnprintf_s(buffer, sizeof(buffer), _TRUNCATE, fmt, args);

  if (level <= logLevel) {
    Message(buffer);
  }

  if (level <= printLevel) {
    OutputDebugStringA(buffer);
    OutputDebugStringA("\n");
  }
}
