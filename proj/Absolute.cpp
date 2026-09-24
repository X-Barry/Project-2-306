#include "Absolute.h"

#include <bitset>
#include <Arduino.h>

/* Absolute encoder using signals from LED/PT pairs for position sensing of a rotary motor */

// Global Variables
std::bitset<5> bitPosition;

// Reads the voltage value of the analog pins 0-4, compares it with a threshold and creates a 5 bit greycode number (1 for rach pin above threshhold 0 for each pin below threshold).
void Absolute::readAnalogPins()
{
    for (int i = 0; i < 4; i++)
    {
        int pinValue = analogRead(A0 + i);
        if (pinValue > threshold)
        {
            bitPosition.set(i);
        }
        else
        {
            bitPosition.reset(i);
        }
    }
    return;
}

// Converts greycode number to a 5 bit binary number.
void Absolute::greyToBinary()
{
    std::bitset<5> temp = bitPosition;
    while ((temp >>= 1).any())
    {
        bitPosition ^= temp;
    }
    return;
}

// Converts the 5 bit binary number to a 0-360 degree angle. And compares it with intial to detect CW or CCW rotation.
void Absolute::binaryToAngle()
{
    float angleResolution = 360.0 / 2 ^ resolution; // 32 positions for 5 bit binary number
    float currentAngle = bitPosition.to_ulong() * angleResolution;

    // This section of code does not work as intended, need to figure out wraparound case
    // Determine rotation direction
    if (currentAngle - previousPos > 0)
    {
        direction = Direction::CW;
    }
    else if (currentAngle - previousPos < 0)
    {
        direction = Direction::CCW;
    }
    else
    {
        direction = Direction::NONE;
    }

    displacement = currentAngle - previousPos;
    previousPos = currentAngle;
    currentPos = currentAngle;
    return;
}