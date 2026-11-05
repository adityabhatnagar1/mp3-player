#pragma once
// Minimal host-side stand-in for Arduino.h so firmware logic can run on a PC.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define F(x) x
uint32_t millis();
class SimSerial {
 public:
  void   setRxBufferSize(size_t) {}
  void   begin(uint32_t) {}
  int    available();
  size_t read(uint8_t* buf, size_t n);
  size_t write(const uint8_t* buf, size_t n);
  size_t print(const char* s)      { return write((const uint8_t*)s, strlen(s)); }
  size_t println(const char* s)    { size_t n = print(s); n += print("\r\n"); return n; }
  size_t print(unsigned long v)    { char b[24]; snprintf(b, sizeof b, "%lu", v); return print(b); }
  size_t println(unsigned long v)  { size_t n = print(v); n += print("\r\n"); return n; }
  size_t print(unsigned v)         { return print((unsigned long)v); }
  size_t println(unsigned v)       { return println((unsigned long)v); }
};
extern SimSerial Serial;
