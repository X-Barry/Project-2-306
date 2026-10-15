/*
  RawSignalLogger - logs the raw phototransistor readings on A0-A5 while the disk turns.

  Use it for:
    - the waveform figures (raw signal from every LED/PT pair)
    - choosing the resistor and threshold for each channel (min / max / midpoint are printed)
    - the phase offset and duty cycle between quadrature channels A and B

  How to use (Arduino IDE):
    1. Upload, open the Serial Monitor at 250000 baud.
    2. Type a motor PWM and press Enter:
         40   spins the motor slowly one way while logging
         -40  spins it the other way
         0    motor off, turn the disk by hand while logging
       Use the smallest PWM that still turns the motor smoothly, so each slot gets plenty of samples.
    3. After LOG_MS the motor stops and a min / max / midpoint summary is printed for every pin.
    4. Copy everything from the Serial Monitor into a text file in performance_v2/data/.

  Output, one line per sample (t_us = microseconds since the capture started):
    RAW,t_us,A0,A1,A2,A3,A4,A5
  Lines starting with # are comments and are ignored by analyze.py.
*/

const unsigned long LOG_MS = 5000; // length of each capture in ms
const int N_PINS = 6;              // A0..A5
const int PWM_PIN = 6;             // motor speed (same as the base codes)
const int DIR_PIN = 3;             // motor direction (same as the base codes)

void printHelp()
{
  Serial.println("# Type a motor PWM (-255..255) and press Enter to log A0-A5 for 5 s.");
  Serial.println("# Small values (e.g. 40) turn the disk slowly; 0 = motor off, turn the disk by hand.");
}

void setup()
{
  Serial.begin(250000);
  printHelp();
}

void loop()
{
  if (Serial.available() == 0)
  {
    return; // wait for a command
  }

  int pwm = Serial.readString().toInt();
  pwm = constrain(pwm, -255, 255);

  analogWrite(DIR_PIN, pwm < 0 ? 255 : 0); // same direction convention as the base codes
  analogWrite(PWM_PIN, abs(pwm));

  int minimum[N_PINS], maximum[N_PINS];
  for (int i = 0; i < N_PINS; i++)
  {
    minimum[i] = 1023;
    maximum[i] = 0;
  }

  Serial.print("# capture start, pwm = ");
  Serial.println(pwm);
  Serial.println("# RAW,t_us,A0,A1,A2,A3,A4,A5");

  unsigned long startMs = millis();
  unsigned long startUs = micros();
  while (millis() - startMs < LOG_MS)
  {
    unsigned long tUs = micros() - startUs;
    int value[N_PINS];
    for (int i = 0; i < N_PINS; i++)
    {
      value[i] = analogRead(A0 + i);
      minimum[i] = min(minimum[i], value[i]);
      maximum[i] = max(maximum[i], value[i]);
    }

    Serial.print("RAW,");
    Serial.print(tUs);
    for (int i = 0; i < N_PINS; i++)
    {
      Serial.print(',');
      Serial.print(value[i]);
    }
    Serial.println();
  }

  analogWrite(PWM_PIN, 0); // stop the motor

  Serial.println("# capture end. pin: min, max, midpoint (a threshold halfway between the two levels)");
  for (int i = 0; i < N_PINS; i++)
  {
    Serial.print("# A");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(minimum[i]);
    Serial.print(", ");
    Serial.print(maximum[i]);
    Serial.print(", ");
    Serial.println((minimum[i] + maximum[i]) / 2);
  }
  printHelp();
}
