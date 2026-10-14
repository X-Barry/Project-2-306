/*
  QuadraturePerformance - automated performance test of the quadrature encoder.

  Runs BaseCodeOne's 15 s speed test RUNS times for every speed in COMMANDS[], one after the
  other, without resetting the board. The PI controller and the built-in encoder counting are
  copied from BaseCodeOne unchanged, so the motor behaves as in the demo. Only the 100 ms
  "spontaneous speed" print is left out, to keep the output short.

  How to use (Arduino IDE):
    1. Upload, open the Serial Monitor at 250000 baud.
    2. Type 0 and press Enter to run the full sweep (+-40 to +-320 RPM, 3 runs each, about
       15 minutes), or type one speed (e.g. -120) to test only that command.
    3. When "# done" appears, copy everything from the Serial Monitor into a text file in
       performance/data/ and run analyze.py.

  Output, one line every 5 s of each run ("own" = our quadrature encoder, error = own - builtin):
    QUAD,cmd_rpm,run,t_s,dir_cmd,dir_builtin,dir_own,builtin_rpm,own_rpm,error_rpm
  t_s is the end of the 5 s window: t_s = 10 is the 5-10 s window that the demo assesses.

  The optical encoder section below is a copy of proj/Quadrature. Copy it again if that changes.
*/

// ---- test settings ----
const int COMMANDS[] = {40, -40, 80, -80, 120, -120, 160, -160, 200, -200, 240, -240, 280, -280, 320, -320};
const int N_COMMANDS = sizeof(COMMANDS) / sizeof(COMMANDS[0]);
const int RUNS = 3;                  // runs per command
const unsigned long PAUSE_MS = 1500; // rest between runs so the motor stops

// ---- BaseCodeOne variables ----
// unsigned long instead of BaseCodeOne's int: an int overflows after 32.7 s and a sweep runs for minutes
unsigned long b = 0; // reading the time for main loop to be run for 15s
unsigned long c = 0; // memory for the time in mainloop

float s = 0; // built-in encoder counts
float s_2;   // built-in encoder counts for RPM calculation for PI controler

float rpmm; // rpm obtained each 100ms from built-in encoder

int s1 = 0;         // built-in encoder chanel one outpot
int s2 = 0;         // built-in encoder chanel two outpot
int r = 0;          // repetition indicator for reading counts of bult-in encoder
int s2m = 0;        // memory of built-in encoder chanel two outpot
int directionm = 0; // indicator for direction read by built-in encoder
int dirm;           // indicator for direction read by built-in encode
int RPM;            // Commanded RPM

float ctrl;    // PI controller outpot
float kp = .4; // proportional gain of PI controller
float ki = .01; // integral gain of PI controller
float eri;     // integral of error of PI controller

int repc = 1;         // repetition condition of PI controller
unsigned long t0;     // memory of time for the Purpose of displaying the results
int repeat = 0;       // repeat indicator to only let the memory of time be updated once

// ---- optical quadrature encoder (copy of proj/Quadrature) ----
const int OPTICAL_PIN_A = A4;
const int OPTICAL_PIN_B = A5;
const unsigned int OPTICAL_SLOTS = 16;

const int A_LOW = 20;
const int A_HIGH = 70;
const int B_LOW = 20;
const int B_HIGH = 70;

const int OPTICAL_DIRECTION_SIGN = +1;

bool opticalA = false;
bool opticalB = false;
bool previousOpticalA = false;

unsigned long opticalPulses = 0; // # Channel A rising edges in window
long opticalDirectionVotes = 0;  // determines dominent measured direction

bool applyHysteresis(int raw, bool previous, int low, int high)
{
  if (raw >= high)
  {
    return true;
  }
  if (raw <= low)
  {
    return false;
  }
  return previous;
}

void initialiseOpticalEncoder() // determines starting state so not miscounted
{
  const int midpointA = (A_LOW + A_HIGH) / 2;
  const int midpointB = (B_LOW + B_HIGH) / 2;

  opticalA = analogRead(OPTICAL_PIN_A) >= midpointA;
  opticalB = analogRead(OPTICAL_PIN_B) >= midpointB;

  previousOpticalA = opticalA;

  opticalPulses = 0;
  opticalDirectionVotes = 0;
}

void updateOpticalEncoder()
{
  // read the two phototransistor outputs
  const int rawA = analogRead(OPTICAL_PIN_A);
  const int rawB = analogRead(OPTICAL_PIN_B);

  // Convert the analogue readings into logical states
  opticalA = applyHysteresis(rawA, opticalA, A_LOW, A_HIGH);
  opticalB = applyHysteresis(rawB, opticalB, B_LOW, B_HIGH);

  // Detect a LOW-to-HIGH transition on Channel A --> was Low now is High
  if (opticalA && !previousOpticalA)
  {
    opticalPulses++;

    int edgeDirection;

    // Inspect B at the detected A rising edge
    if (!opticalB)
    {
      edgeDirection = +1;
    }
    else
    {
      edgeDirection = -1;
    }

    opticalDirectionVotes += OPTICAL_DIRECTION_SIGN * edgeDirection;
  }

  // Save A's current state for the next reading
  previousOpticalA = opticalA;
}

const char *opticalDirection()
{
  if (opticalPulses == 0)
  {
    return "NONE";
  }
  if (opticalDirectionVotes > 0)
  {
    return "CW";
  }
  if (opticalDirectionVotes < 0)
  {
    return "CCW";
  }
  return "UNCLEAR";
}
// ---- end of optical quadrature encoder ----

// Prints one QUAD line for the 5 s window that just ended.
void printWindow(int command, int run, unsigned long windowEnd)
{
  float builtinRPM = (s / (228)) * 12; // formula for rpm in each 5s
  float opticalRPM = (opticalPulses / (float)OPTICAL_SLOTS) * 12.0;

  Serial.print("QUAD,");
  Serial.print(command);
  Serial.print(',');
  Serial.print(run);
  Serial.print(',');
  Serial.print(windowEnd / 1000);
  Serial.print(',');
  Serial.print(command > 0 ? "CW" : "CCW");
  Serial.print(',');
  Serial.print(dirm == 0 ? "CW" : "CCW");
  Serial.print(',');
  Serial.print(opticalDirection());
  Serial.print(',');
  Serial.print(builtinRPM, 2);
  Serial.print(',');
  Serial.print(opticalRPM, 2);
  Serial.print(',');
  Serial.println(opticalRPM - builtinRPM, 2);
}

// One 15 s run, copied from BaseCodeOne's loop() with the optical encoder added.
void runOnce(int command, int run)
{
  RPM = abs(command);
  analogWrite(3, command < 0 ? 255 : 0); // changing the direction of motor's rotation

  // start every run from the same state as a freshly reset board
  s = 0;
  s_2 = 0;
  rpmm = 0;
  eri = 0;
  repc = 1;
  repeat = 0;
  directionm = 0;
  dirm = 0;
  initialiseOpticalEncoder();

  b = millis(); // reading time
  c = b;        // storing the current time

  while ((b >= c) && (b <= (c + 15500))) // let the main loop to be run for 15s
  {
    if (b % 13 == 0 && repc == 1) // PI controller
    {
      eri = ki * (RPM - rpmm) + eri;
      ctrl = 50 + kp * (RPM - rpmm) + eri;
      analogWrite(6, ctrl);
      repc = 0;
    }
    if (b % 13 == 1)
    {
      repc = 1;
    }

    s1 = digitalRead(7); // reading Chanel 1 of builtin encoder
    s2 = digitalRead(8); // reading Chanel 2 of builtin encoder

    updateOpticalEncoder();

    if (s1 != s2 && r == 0)
    {
      s = s + 1;     // counters for rpm that displyed every 5s
      s_2 = s_2 + 1; // counters for rpm that used in PI contoller
      r = 1;
    }

    if (s1 == s2 && r == 1)
    {
      s = s + 1;
      s_2 = s_2 + 1;
      r = 0;
    }

    b = millis(); // updating time
    // BaseCodeOne uses b%100<=1 here; that can start t0 at a ...01 ms time, and then the
    // 5 s windows below never line up, so this waits for an exact multiple of 100 ms.
    if (b % 100 == 0 && repeat == 0)
    {
      t0 = b; // storing the current time once
      repeat = 1;
    }

    if (b % 100 == 0)
    {
      rpmm = (s_2 / (2 * 114)) * 600; // formulation for rpm in each 100ms for PI controller
      s_2 = 0;                        // reseting the counters of PI controller rpm meter

      if ((b - t0) % 5000 == 0) // every 5 s from t0 (BaseCodeOne also prints at t0 itself; that line is skipped)
      {
        if (b != t0)
        {
          printWindow(command, run, b - t0);
        }
        s = 0;
        directionm = 0;
        opticalPulses = 0;
        opticalDirectionVotes = 0;
      }
      delay(1);
    }

    if ((s1 == HIGH) && (s2 == HIGH) && (s2m == LOW)) // reading the direction of motor by cheaking which chanel follows which
    {
      directionm = directionm + 1;
    }

    if ((s1 == LOW) && (s2 == LOW) && (s2m == HIGH))
    {
      directionm = directionm + 1;
    }

    s2m = s2; // memory of the previous builtin encoder chanel 2

    if (directionm > 100)
    {
      dirm = 0;
    }
    if (directionm < 20)
    {
      dirm = 1;
    }

    b = millis(); // updating time
  }
  analogWrite(6, 0); // turning off the motor
}

void printHelp()
{
  Serial.println("# Type 0 and press Enter for the full sweep (+-40 to +-320 RPM, 3 runs each, ~15 min),");
  Serial.println("# or type one speed (e.g. -120) to test only that command.");
}

void runCommand(int command)
{
  Serial.print("# command ");
  Serial.print(command);
  Serial.println(" RPM");
  for (int run = 1; run <= RUNS; run++)
  {
    runOnce(command, run);
    delay(PAUSE_MS);
  }
}

void setup()
{
  Serial.begin(250000); // Baud rate of communication
  printHelp();
}

void loop()
{
  if (Serial.available() == 0)
  {
    return; // wait for a command
  }

  int command = Serial.readString().toInt();
  Serial.println("# QUAD,cmd_rpm,run,t_s,dir_cmd,dir_builtin,dir_own,builtin_rpm,own_rpm,error_rpm");
  if (command == 0)
  {
    for (int i = 0; i < N_COMMANDS; i++)
    {
      runCommand(COMMANDS[i]);
    }
  }
  else
  {
    runCommand(command);
  }
  Serial.println("# done");
  printHelp();
}
