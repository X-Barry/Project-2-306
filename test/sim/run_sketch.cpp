// Runs the real sketch (setup() then loop() forever) on the simulated rig.
// Usage: run_sketch <wheel.dxf> <degrees typed into Serial> <seconds to simulate>
#include <cstdio>
#include "Arduino.h"
#include SKETCH_INO // set by run.sh

namespace sim { void load_wheel(const char *); }

int main(int argc, char **argv)
{
  sim::load_wheel(argv[1]);
  sim::serial_in = argc > 2 ? argv[2] : "90";
  sim::limit_us = (unsigned long long)((argc > 3 ? atof(argv[3]) : 60) * 1e6);
  sim::log_samples = true;
  printf("[sim] typing \"%s\" into the Serial Monitor, simulating %.0f s\n", sim::serial_in.c_str(), sim::limit_us * 1e-6);
  setup();
  for (;;) loop();
}
