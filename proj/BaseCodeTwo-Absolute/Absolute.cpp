#include "Absolute.h"

#include <Arduino.h>

/* Absolute encoder using signals from LED/PT pairs for position sensing of a rotary motor */

// Constructor for Absolute class, initializes the current position, threshold, and resolution.
// The sensors are not read here: a global object is constructed before the Arduino core has
// started the ADC, so analogRead() would return garbage. Call update() from setup() instead.
Absolute::Absolute(int threshold, int resolution)
{
    this->threshold = threshold;
    this->resolution = resolution;
    this->bitPosition = 0;
    this->hasReading = false;
    this->currentPos = 0.0;
    this->previousPos = 0.0;
    this->totalDisplacement = 0.0;
    this->rejected = 0;
    this->lastReading = 0.0;
    this->repeats = 0;
}

// Starts the process of reading the analog pins, converting to greycode, then to binary, and finally to an angle.
void Absolute::update()
{
    readAnalogPins();
    greyToBinary();
    binaryToAngle();
}

// Zeroes the displacement so it is measured from the current position.
void Absolute::resetDisplacement()
{
    totalDisplacement = 0.0;
    rejected = 0;
}

// Reads the voltage value of the analog pins A1-A5, compares it with a threshold and creates a 5 bit greycode number.
// The wheel (wheel_gray5.dxf) is drawn with the WHITE sectors as logic 1, and white reads BELOW the threshold,
// so a pin below threshold sets the bit. A1 is the LSB, A5 the MSB.
void Absolute::readAnalogPins()
{
    for (int i = 0; i < 5; i++)
    {
        int pinValue = analogRead(A1 + i);
        if (pinValue < threshold)
        {
            bitPosition |= (1 << i);
        }
        else
        {
            bitPosition &= ~(1 << i);
        }
    }
    return;
}

// Converts greycode number to a 5 bit binary number.
void Absolute::greyToBinary()
{
    for (uint8_t temp = bitPosition >> 1; temp; temp >>= 1)
    {
        bitPosition ^= temp;
    }
    return;
}

// Converts the 5 bit binary number to a 0-360 degree angle, and adds the step since the previous
// accepted angle to the total displacement (CW positive). The step is taken as the shorter way round.
// Glitches (e.g. every sensor reading dark for one sample) must not be counted: out and back, a glitch
// exactly 180 deg away is counted as -180 both ways, adding a whole extra turn each time. So a reading
// is only used once two updates in a row agree on it, and a step bigger than MAX_STEP_DEG is ignored.
void Absolute::binaryToAngle()
{
    float angleResolution = 360.0 / (1 << resolution); // 32 positions for 5 bit binary number
    float reading = bitPosition * angleResolution;

    if (!hasReading)
    {
        // first reading: nothing to compare against yet
        currentPos = previousPos = lastReading = reading;
        repeats = 1;
        hasReading = true;
        return;
    }

    repeats = reading == lastReading ? repeats + 1 : 1;
    lastReading = reading;

    float step = fmod(reading - currentPos + 360.0, 360.0); // 0-359.99, keeps the fractional degrees
    if (step > 180)
    {
        step -= 360; // angle decreased (e.g. CCW)
    }
    bool far = step > MAX_STEP_DEG || step < -MAX_STEP_DEG;

    if (step > angleResolution || step < -angleResolution)
    {
        rejected++; // further than the next sector: not a normal sector change
    }
    if (repeats < 2 || (far && repeats < CONFIRM_READS))
    {
        return; // not confirmed yet: keep the last good angle
    }

    previousPos = currentPos;
    currentPos = reading;
    if (!far)
    {
        totalDisplacement += step; // a far reading that keeps coming back moves the angle, but isn't counted as motion
    }
    return;
}
