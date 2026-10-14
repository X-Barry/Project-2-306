// The simulated rig: wheel (loaded from the real DXF), 5 LED/PT pairs, gear motor, built-in
// quadrature encoder, a virtual clock, and the Arduino calls that read/drive them.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <algorithm>
#include "Arduino.h"

namespace sim
{
  // ---- calibration knobs: set these from TestScript.ino readings / the motor datasheet ----
  double black_counts = 900;     // analogRead with a black sector over the PT (black reads high)
  double white_counts = 60;      // analogRead with a white sector over the PT (light gets through)
  double max_rpm = 60;           // output shaft speed at PWM 255
  double deadband_pwm = 20;      // PWM at or below this doesn't turn the motor
  double tau_s = 0.05;           // motor speed time constant
  int counts_per_rev = 228;      // built-in encoder edges per shaft revolution (as the sketch assumes)
  int pin_lsb = A1;              // A1 = bit 0 (outer track) ... A5 = bit 4 (inner track)
  bool forward_increases = true; // pin 3 LOW spins the wheel so its position counts up
  bool log_samples = false;
  int dark_reads = 0;            // fault injection: the next N analogRead() calls read black (all sensors dark)

  const uint8_t PIN_PWM = 6, PIN_DIR = 3, PIN_ENC_A = 7, PIN_ENC_B = 8;
  double shaft_deg = 0, speed_dps = 0;
  unsigned long long now_us = 0, limit_us = ~0ULL;
  std::string serial_in;
  size_t serial_pos = 0;
  int pwm = 0, dir = 0;

  struct Sector { double r0, r1, a0, sweep; };
  std::vector<Sector> clear_sectors;
  std::vector<double> track_r; // sensor radius of each track, inner -> outer

  static double norm(double a) // -> [0, 2pi)
  {
    a = fmod(a, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a > 2 * M_PI - 1e-6 ? 0 : a;
  }

  void load_wheel(const char *path)
  {
    std::ifstream f(path);
    if (!f) { fprintf(stderr, "[sim] can't open wheel file %s\n", path); exit(2); }
    std::string code, val, layer;
    std::vector<std::pair<double, double>> verts, radii;
    bool in_vertex = false;
    auto trim = [](std::string &s) {
      while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
      s.erase(0, s.find_first_not_of(' '));
    };
    while (std::getline(f, code) && std::getline(f, val))
    {
      trim(code); trim(val);
      int c = atoi(code.c_str());
      if (c == 0)
      {
        in_vertex = val == "VERTEX";
        if (val == "POLYLINE") { verts.clear(); layer.clear(); }
        if (in_vertex) verts.push_back({0, 0});
        if (val == "SEQEND" && verts.size() == 4)
        { // annular sector: outer start, outer end, inner end, inner start (Y flipped, as written)
          double r0 = 1e9, r1 = 0;
          for (auto &v : verts) { double r = hypot(v.first, v.second); r0 = std::min(r0, r); r1 = std::max(r1, r); }
          double a0 = norm(atan2(-verts[0].second, verts[0].first));
          double a1 = norm(atan2(-verts[1].second, verts[1].first));
          radii.push_back({r0, r1});
          if (layer == "CLEAR") clear_sectors.push_back({r0, r1, a0, norm(a1 - a0)});
        }
      }
      else if (c == 8 && !in_vertex) layer = val;
      else if (in_vertex && c == 10) verts.back().first = atof(val.c_str());
      else if (in_vertex && c == 20) verts.back().second = atof(val.c_str());
    }
    std::sort(radii.begin(), radii.end());
    for (auto &r : radii)
      if (track_r.empty() || fabs((r.first + r.second) / 2 - track_r.back()) > 1e-3)
        track_r.push_back((r.first + r.second) / 2);
    if (track_r.size() != 5 || clear_sectors.empty())
    { fprintf(stderr, "[sim] expected 5 tracks in %s, found %zu\n", path, track_r.size()); exit(2); }
  }

  int positions() { return 1 << track_r.size(); }
  int true_position() { return (int)floor(norm(shaft_deg * M_PI / 180) / (2 * M_PI) * positions()); }

  static bool is_clear(int track, double a)
  {
    double r = track_r[track];
    for (auto &s : clear_sectors)
      if (r >= s.r0 && r < s.r1 && norm(a - s.a0) < s.sweep) return true;
    return false;
  }

  static void advance(unsigned long long us)
  {
    while (us)
    {
      unsigned long long step = std::min(us, 100ULL);
      double dt = step * 1e-6;
      double mag = pwm <= deadband_pwm ? 0 : (pwm - deadband_pwm) / (255 - deadband_pwm) * max_rpm * 6;
      double target = mag * (dir ? -1 : 1) * (forward_increases ? 1 : -1);
      speed_dps += (target - speed_dps) * std::min(1.0, dt / tau_s);
      shaft_deg += speed_dps * dt;
      now_us += step;
      us -= step;
      if (now_us >= limit_us)
      {
        printf("\n[sim] stopped at the %.1f s time limit (true shaft angle %.2f deg)\n", now_us * 1e-6, shaft_deg);
        fflush(stdout);
        exit(0);
      }
    }
  }
}

using namespace sim;

int analogRead(uint8_t pin)
{
  advance(112); // a real Uno conversion takes ~112 us
  if (pin < A0) pin += A0;  // analogRead(1) == analogRead(A1)
  int bit = pin - pin_lsb;
  if (track_r.empty())
  {
    static bool warned = false;
    if (!warned) printf("[sim] WARNING analogRead() before setup(): a global object's constructor runs before "
                        "the Arduino core starts the ADC, so this reading is garbage on a real board\n");
    warned = true;
    return 0;
  }
  if (bit < 0 || bit >= (int)track_r.size()) return 0;
  if (dark_reads > 0) { dark_reads--; return (int)black_counts; }
  return is_clear((int)track_r.size() - 1 - bit, shaft_deg * M_PI / 180) ? (int)white_counts : (int)black_counts;
}

void analogWrite(uint8_t pin, int val)
{
  advance(6);
  int eff = val == 0 ? 0 : val == 255 ? 255 : (val & 0xFF); // what wiring_analog.c does with the 8-bit timer
  if (eff != val)
  {
    static int warnings = 0;
    if (warnings++ < 5)
      printf("[sim %8.3f s] WARNING analogWrite(%d, %d): a real Uno outputs %d (only the low 8 bits are used)\n",
             now_us * 1e-6, pin, val, eff);
  }
  if (pin == PIN_PWM) pwm = eff;
  if (pin == PIN_DIR) dir = eff > 127;
}

int digitalRead(uint8_t pin)
{
  advance(5);
  double cycles = shaft_deg / 360.0 * counts_per_rev / 4; // 4 edges per quadrature cycle
  double ph = cycles - floor(cycles);
  if (pin == PIN_ENC_A) return ph < 0.5;
  if (pin == PIN_ENC_B) return fmod(ph + 0.25, 1.0) < 0.5;
  return LOW;
}

void digitalWrite(uint8_t pin, uint8_t val)
{
  advance(5);
  if (pin == PIN_PWM) pwm = val ? 255 : 0;
  if (pin == PIN_DIR) dir = val;
}

void pinMode(uint8_t, uint8_t) { advance(2); }
unsigned long millis() { advance(2); return (unsigned long)(now_us / 1000); }
unsigned long micros() { advance(2); return (unsigned long)now_us; }
void delay(unsigned long ms) { advance(ms * 1000ULL); }

// ---- Serial: output to the console with the virtual time, input from the command line ----
SerialPort Serial;
static bool line_start = true;

int SerialPort::available() { advance(10); return (int)(serial_in.size() - serial_pos); }
String SerialPort::readString()
{
  advance(1000000); // the real readString() waits out its 1 s timeout
  String s(serial_in.substr(serial_pos));
  serial_pos = serial_in.size();
  return s;
}
void SerialPort::print(const char *s)
{
  for (; *s; s++)
  {
    if (line_start && log_samples) // tag each line with the true shaft position to compare against
      printf("[%8.3f s | true shaft %8.2f deg, pos %2d] ", now_us * 1e-6, shaft_deg, true_position());
    else if (line_start)
      printf("[%8.3f s] ", now_us * 1e-6);
    putchar(*s);
    line_start = *s == '\n';
    advance(40); // 250000 baud
  }
}
void SerialPort::print(char c) { char s[2] = {c, 0}; print(s); }
void SerialPort::print(long v) { char b[24]; snprintf(b, sizeof b, "%ld", v); print(b); }
void SerialPort::print(unsigned long v) { char b[24]; snprintf(b, sizeof b, "%lu", v); print(b); }
void SerialPort::print(double v, int digits) { char b[48]; snprintf(b, sizeof b, "%.*f", digits, v); print(b); }
