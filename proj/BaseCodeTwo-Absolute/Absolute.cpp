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
// reading to the total displacement (CW positive). The step is taken as the shorter way round, so
// the shaft must turn less than 180 degrees between two update() calls.
void Absolute::binaryToAngle()
{
    previousPos = currentPos;
    float angleResolution = 360.0 / (1 << resolution); // 32 positions for 5 bit binary number
    currentPos = bitPosition * angleResolution;

    if (!hasReading)
    {
        // first reading: nothing to compare against yet
        previousPos = currentPos;
        hasReading = true;
        return;
    }

    float diff = fmod(currentPos - previousPos + 360.0, 360.0); // 0–359.99, keeps the fractional degrees

    if (diff < 180)
    {
        // angle increased by diff degrees (e.g. CW), or no movement when diff == 0
        totalDisplacement += diff;
    }
    else
    {
        // angle decreased by (360 - diff) degrees (e.g. CCW)
        totalDisplacement -= 360 - diff;
    }
    return;
}