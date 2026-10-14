/*
   Header file for the Absolute class, which provides functionality to read analog pin
   values, convert them to a greycode number, and then convert that number to a binary
   representation and finally to a degree angle.

   The wheel only knows where it is within one turn, so update() must be called often enough
   that the shaft turns less than 180 degrees between calls. Each update() adds the small step
   since the previous call to a running total, so displacements of any size (including several
   full turns) are tracked. resetDisplacement() zeroes that total.

   Sensor glitches are filtered: a new reading is only used once two update() calls in a row agree
   on it. A reading more than MAX_STEP_DEG away from the last accepted angle can't be real motion at
   the update rate the sketch uses, so it is ignored until it has come back CONFIRM_READS times in a
   row; then the angle follows it, but the jump is not added to the displacement. Readings further
   than one sector from the last accepted angle are counted in getRejected().
*/

#pragma once

#include <stdint.h>

enum class Direction
{
   CW = 1,
   CCW = -1,
   NONE = 0
};

class Absolute
{
public:
   // Constructor. Does not read the sensors: call update() once from setup() (the ADC is not
   // running yet when a global object is constructed).
   Absolute(int threshold, int resolution);
   // Destructor
   ~Absolute() {};
   // Reads the pins, recalculates the angle and adds the step since the last call to the displacement
   void update();
   // Zeroes the displacement so it counts from the current position
   void resetDisplacement();
   // Getters
   float getAngle()
   {
      return currentPos;
   }
   // Size of the displacement since the last resetDisplacement(), in degrees (always >= 0)
   float getDisplacement()
   {
      return totalDisplacement < 0 ? -totalDisplacement : totalDisplacement;
   }
   // Direction of the displacement since the last resetDisplacement()
   Direction getDirection()
   {
      return totalDisplacement > 0 ? Direction::CW : totalDisplacement < 0 ? Direction::CCW : Direction::NONE;
   }
   // Number of readings further than one sector from the last accepted angle since the last resetDisplacement()
   int getRejected()
   {
      return rejected;
   }

   static constexpr float MAX_STEP_DEG = 45.0; // largest believable move between two update() calls
   static const int CONFIRM_READS = 3;         // identical far readings needed to accept a jump

private:
   int threshold;
   int resolution;
   uint8_t bitPosition; // one bit per LED/PT pair, bit 0 = A1 ... bit 4 = A5
   bool hasReading;     // false until the first update(), so the first reading isn't counted as a move
   float currentPos;
   float previousPos;
   float totalDisplacement; // signed, CW positive, since the last resetDisplacement()
   int rejected;            // suspicious readings since the last resetDisplacement()
   float lastReading;       // latest raw reading, and how many updates in a row have given it
   int repeats;
   void readAnalogPins();
   void greyToBinary();
   void binaryToAngle();
};
