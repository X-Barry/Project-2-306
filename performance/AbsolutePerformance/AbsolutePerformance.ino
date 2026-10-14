/*
  AbsolutePerformance - automated performance test of the absolute encoder.

  Runs BaseCodeTwo's positioning test (10 moves of 4 s each) for every angle in COMMANDS[],
  one after the other, without resetting the board. The PI controller and the built-in encoder
  counting are copied from BaseCodeTwo unchanged, so the motor behaves as in the demo.

  How to use (Arduino IDE):
    1. Upload, open the Serial Monitor at 250000 baud.
    2. Type 0 and press Enter to run the full sweep (+-10 to +-90 deg, about 15 minutes),
       or type one angle (e.g. -30) to test only that command.
    3. When "# done" appears, copy everything from the Serial Monitor into a text file in
       performance/data/ and run analyze.py.

  Output, one line per move ("own" = our absolute encoder, error = own - builtin):
    ABS,cmd_deg,rep,dir_cmd,dir_own,builtin_deg,own_deg,error_deg,own_pos_deg
  A summary per command (mean / std / max error and the demo's best-8-of-10 average) is printed
  as # comment lines, which analyze.py ignores.

  Absolute.h / Absolute.cpp are copies of proj/BaseCodeTwo-Absolute/ (the Arduino IDE only
  compiles files inside the sketch folder). Copy them again if the originals change.
*/

#include "Absolute.h"

// ---- test settings ----
const int COMMANDS[] = {10, -10, 20, -20, 30, -30, 40, -40, 50, -50, 60, -60, 70, -70, 80, -80, 90, -90};
const int N_COMMANDS = sizeof(COMMANDS) / sizeof(COMMANDS[0]);
const int REPS = 10;                 // moves per command, as in BaseCodeTwo
const int OWN_DIRECTION_SIGN = +1;   // set to -1 if dir_own always comes out opposite to dir_cmd
const unsigned long PAUSE_MS = 1000; // rest between commands

// ---- BaseCodeTwo variables ----
float deg = 45; // Rotation degree
float s = 0;    // Encoder counts
int sm1 = 0;    // Built-in chanel 1
int sm2 = 0;    // Built-in chanel 2
int r = 0;      // indicator for reading builtin encoder to avoid the reading redundancy
float er;       // Proportional error for PI controller
float eri;      // Integral error for PI controller

// unsigned long instead of BaseCodeTwo's int: an int overflows after 32.7 s and a sweep runs for minutes
unsigned long t = 0;    // time in ms
unsigned long t0 = 0;   // memory for time in ms
unsigned long tAbs = 0; // time of the last absolute sensor reading in ms

// BaseCodeTwo works out kp before the angle is typed in, so it always uses its default deg = 45.
// Using the same constant here keeps the motor's behaviour identical to the demo.
const float kp = .6 * 90 / 45; // proportional gain of PI
const float ki = .02;          // integral gain of PI

Absolute absolute(100, 5); // threshold=100, resolution=5 (pins A1-A5)

float errors[REPS]; // errors of the current command, for the summary

// One 4 s move, copied from BaseCodeTwo's loop(). Returns after the motor has stopped.
void positionOnce()
{
  absolute.update();            // reading the absolute sensor at the start of the run
  absolute.resetDisplacement(); // measuring this run's displacement from here

  t = millis();
  t0 = t;
  tAbs = t;
  while (t < t0 + 4000) // 4 seconds per move
  {
    if (t - tAbs >= 5) // reading the absolute sensor every 5ms
    {
      absolute.update();
      tAbs = t;
    }

    if (t % 10 == 0) // PI controller that runs every 10ms
    {
      if (s < deg * 114 * 2 / 360)
      {
        er = deg - s * 360 / 228;
        eri = eri + er;
        analogWrite(6, kp * er + ki * eri);
      }

      if (s >= deg * 228 / 360)
      {
        analogWrite(6, 0);
        eri = 0;
      }
      delay(1);
    }

    sm1 = digitalRead(7); // reading chanel 1
    sm2 = digitalRead(8); // reading chanel 2

    if (sm1 != sm2 && r == 0)
    { // counting the number changes for both chanels
      s = s + 1;
      r = 1;
    }
    if (sm1 == sm2 && r == 1)
    {
      s = s + 1;
      r = 0;
    }

    t = millis(); // updating time
  }

  analogWrite(6, 0); // turning off the motor before the final reading
  absolute.update(); // final absolute sensor reading, at the same moment the builtin encoder stopped counting
  delay(500);
}

const char *directionName(int sign)
{
  return sign > 0 ? "CW" : sign < 0 ? "CCW" : "NONE";
}

void printSummary(int directionCorrect);

// Runs REPS moves of `command` degrees and prints one ABS line per move plus a summary.
void runCommand(int command)
{
  deg = abs(command);
  analogWrite(3, command < 0 ? 255 : 0); // pin 3 sets the direction, as in BaseCodeTwo
  s = 0;
  eri = 0;

  Serial.print("# command ");
  Serial.print(command);
  Serial.println(" deg");

  int directionCorrect = 0;
  for (int rep = 1; rep <= REPS; rep++)
  {
    positionOnce();

    float builtin = s * 360 / 228; // every full revolution is 228 counts of the builtin encoder
    float own = absolute.getDisplacement();
    float error = own - builtin;
    int ownSign = OWN_DIRECTION_SIGN * (int)absolute.getDirection();
    int cmdSign = command > 0 ? 1 : -1;
    errors[rep - 1] = error;
    if (ownSign == cmdSign)
    {
      directionCorrect++;
    }

    Serial.print("ABS,");
    Serial.print(command);
    Serial.print(',');
    Serial.print(rep);
    Serial.print(',');
    Serial.print(directionName(cmdSign));
    Serial.print(',');
    Serial.print(directionName(ownSign));
    Serial.print(',');
    Serial.print(builtin, 2);
    Serial.print(',');
    Serial.print(own, 2);
    Serial.print(',');
    Serial.print(error, 2);
    Serial.print(',');
    Serial.println(absolute.getAngle(), 2);

    s = 0;
  }
  analogWrite(6, 0);

  printSummary(directionCorrect);
}

// Mean, standard deviation, max |error| and the demo's average unsigned error over the best 8 of 10.
void printSummary(int directionCorrect)
{
  float sum = 0, sumSq = 0, maxAbs = 0;
  float absErr[REPS];
  for (int i = 0; i < REPS; i++)
  {
    sum += errors[i];
    sumSq += errors[i] * errors[i];
    absErr[i] = fabs(errors[i]);
    maxAbs = max(maxAbs, absErr[i]);
  }
  float mean = sum / REPS;
  float stdDev = sqrt(max(0.0, (sumSq - REPS * mean * mean) / (REPS - 1)));

  // best 8 of 10: drop the two largest unsigned errors
  for (int i = 1; i < REPS; i++) // insertion sort, smallest first
  {
    float v = absErr[i];
    int j = i - 1;
    while (j >= 0 && absErr[j] > v)
    {
      absErr[j + 1] = absErr[j];
      j--;
    }
    absErr[j + 1] = v;
  }
  int best = REPS > 2 ? REPS - 2 : REPS;
  float bestSum = 0;
  for (int i = 0; i < best; i++)
  {
    bestSum += absErr[i];
  }

  Serial.print("# summary: mean error ");
  Serial.print(mean, 2);
  Serial.print(", std ");
  Serial.print(stdDev, 2);
  Serial.print(", max |error| ");
  Serial.print(maxAbs, 2);
  Serial.print(", best ");
  Serial.print(best);
  Serial.print(" avg |error| ");
  Serial.print(bestSum / best, 2);
  Serial.print(", direction correct ");
  Serial.print(directionCorrect);
  Serial.print('/');
  Serial.println(REPS);
}

void printHelp()
{
  Serial.println("# Type 0 and press Enter for the full sweep (+-10 to +-90 deg, ~15 min),");
  Serial.println("# or type one angle (e.g. -30) to test only that command.");
}

void setup()
{
  absolute.update(); // first absolute sensor reading (home position)
  Serial.begin(250000);
  printHelp();
}

void loop()
{
  if (Serial.available() == 0)
  {
    return; // wait for a command
  }

  int command = Serial.readString().toInt();
  Serial.println("# ABS,cmd_deg,rep,dir_cmd,dir_own,builtin_deg,own_deg,error_deg,own_pos_deg");
  if (command == 0)
  {
    for (int i = 0; i < N_COMMANDS; i++)
    {
      runCommand(COMMANDS[i]);
      delay(PAUSE_MS);
    }
  }
  else
  {
    runCommand(command);
  }
  Serial.println("# done");
  printHelp();
}
