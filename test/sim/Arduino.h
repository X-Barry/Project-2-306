// Host-side stand-in for the Arduino AVR core: just enough of the API for this project,
// so the real Absolute.cpp / .ino compile on a PC and run against a simulated wheel + motor.
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>

typedef uint8_t byte;
typedef bool boolean;
enum { LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2 };
static const uint8_t A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19;  // Uno numbering

// Same macros as the real core (hardware/arduino/avr/cores/arduino/Arduino.h), so a name
// clash that breaks the real build breaks this one too.
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);
void pinMode(uint8_t pin, uint8_t mode);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);

class String : public std::string
{
public:
  String(const std::string &s = "") : std::string(s) {}
  float toFloat() const { return (float)atof(c_str()); }
  long toInt() const { return atol(c_str()); }
};

class SerialPort
{
public:
  void begin(unsigned long) {}
  int available();
  String readString();
  void print(const char *s);
  void print(const String &s) { print(s.c_str()); }
  void print(char c);
  void print(int v) { print((long)v); }
  void print(unsigned int v) { print((unsigned long)v); }
  void print(long v);
  void print(unsigned long v);
  void print(double v, int digits = 2);  // Arduino prints floats with 2 decimals
  void println() { print("\n"); }
  void println(double v, int digits) { print(v, digits); println(); }
  template <class T>
  void println(T v) { print(v); println(); }
};
extern SerialPort Serial;

// ---- simulator controls (not part of Arduino) ----
namespace sim
{
  extern double black_counts, white_counts, max_rpm, deadband_pwm, tau_s;
  extern int counts_per_rev, pin_lsb;
  extern int dark_reads;                   // fault injection: next N analogRead() calls read black
  extern bool forward_increases, log_samples;
  extern double shaft_deg;                 // true shaft angle; + = wheel position counts up
  extern unsigned long long now_us, limit_us;
  extern std::string serial_in;
  void load_wheel(const char *dxf_path);   // reads the CLEAR sectors + track radii from the DXF
  int positions();                         // 2^tracks
  int true_position();                     // wheel position under the sensors right now
}
