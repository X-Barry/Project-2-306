#include "Absolute.h"

float deg = 45; // Rotation degree
float s = 0;    // Encoder counts
int sm1 = 0;    // Built-in chanel 1
int sm2 = 0;    // Built-in chanel 2
int r = 0;      // indicator for reading builtin encoder to avoid the reading redundancy
float er;       // Proportional error for PI controller
float eri;      // Integral error for PI controller

int t = 0;  // time in ms
int t0 = 0; // memory for time in ms
int tAbs = 0; // time of the last absolute sensor reading in ms

int finish = 0;            // finish indicator
int rep = 1;               // Repetition indicator
Absolute absolute(100, 5); // Create an instance of the Absolute class with threshold=100, resolution=5 (pins A1-A5)

bool commandedCCW = false; // direction typed in by the user ("-" for CCW), shown next to the builtin encoder reading
float absErrors[10];       // unsigned error of each of the 10 positionings, for the best 8 of 10 average

void setup()
{
  absolute.update();    // First absolute sensor reading (starting position)
  Serial.begin(250000); // Baud rate of communication

  Serial.println("Enter the desired rotation in degree.");

  while (Serial.available() == 0) // Obtaining data from user
  {
    // Wait for user input
  }

  deg = Serial.readString().toFloat(); // Reading the Input string from Serial port.
  if (deg < 0)
  {
    analogWrite(3, 255); // change the direction of rotation by applying voltage to pin 3 of arduino
    commandedCCW = true;
  }
  deg = abs(deg);
}

float kp = .6 * 90 / deg; // proportional gain of PI
float ki = .02;           // integral gain of PI

void loop()
{
  // put your main code here, to run repeatedly:

  absolute.update();            // reading the absolute sensor at the start of the run
  absolute.resetDisplacement(); // measuring this run's displacement from here

  t = millis();                      // reading time
  t0 = t;                            // sving the current time in memory
  tAbs = t;
  while (t < t0 + 4000 && rep <= 10) // let the code to ran for 4 seconds each with repetitions of 10
  {
    if (t - tAbs >= 5) // reading the absolute sensor every 5ms so it can follow moves of more than 180 degrees
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
      r = 1; // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1)
    {
      s = s + 1;
      r = 0; // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

    t = millis(); // updating time
    finish = 1;   // cghanging finish indicator
  }

  if (finish == 1)
  {                    // this part of the code is for displaying the result
    analogWrite(6, 0); // turning off the motor before the final reading
    absolute.update(); // final absolute sensor reading, at the same moment the builtin encoder stopped counting
    delay(500);        // half second delay
    rep = rep + 1;     // increasing the repetition indicator
    Serial.print("shaft possition from optical absolute sensor from home position: ");
    Serial.println(absolute.getAngle());

    Serial.print("shaft displacement from optical absolute sensor: ");
    Serial.print(absolute.getDisplacement());
    Serial.print(" degrees, Direction: ");
    Serial.println(absolute.getDirection() == Direction::CW    ? "CW"
                   : absolute.getDirection() == Direction::CCW ? "CCW"
                                                               : "NONE"); // displaying the displacement and direction of rotation from absolute sensor

    Serial.print("Shaft displacement from motor's builtin encoder: ");
    Serial.print(s * 360 / 228); // every full Revolution of the shaft is associated with 228 counts of builtin
                                 // encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts
    Serial.print(" degrees, Commanded direction: ");
    Serial.println(commandedCCW ? "CCW" : "CW");

    float Error = absolute.getDisplacement() - s * 360 / 228;
    Serial.print("Error :");
    Serial.println(Error); // displaying error
    Serial.println();

    absErrors[rep - 2] = fabs(Error); // rep was already increased above, so this is positioning number rep - 1
    if (rep > 10)
    {
      // after the 10th positioning: drop the two largest unsigned errors and average the best 8
      float sum = 0, worst = 0, secondWorst = 0;
      for (int i = 0; i < 10; i++)
      {
        sum += absErrors[i];
        if (absErrors[i] > worst)
        {
          secondWorst = worst;
          worst = absErrors[i];
        }
        else if (absErrors[i] > secondWorst)
        {
          secondWorst = absErrors[i];
        }
      }
      Serial.print("Average unsigned error over the best 8 of 10 positionings: ");
      Serial.println((sum - worst - secondWorst) / 8);
      Serial.println();
    }
    s = 0;
    finish = 0;
  }
  analogWrite(6, 0); // turning off the motor
}
