#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <Arduino.h>

//Angles are in relation to standing. Positive is forward, negative is backwards

class Sequence
{
public:
  Sequence();

  uint16_t getLPos();
  uint16_t getRPos();

  void update();
  void testSequence();

  void poseStand(double kP = 1.4);
  void poseBow  (int16_t angle, double kP = 1.4);
  void poseKickL(int16_t angle, double kP = 1.4);
  void poseKickR(int16_t angle, double kP = 1.4);
  void poseStepL(int16_t angle, double kP = 1.2);
  void poseStepR(int16_t angle, double kP = 1.2);
  void poseCustom(int16_t angleL, int16_t angleR, double kP = 1.4);

  void moveWalk(uint8_t stepCount = 1, bool startRight = true);
  void moveWalkSlow(uint8_t stepCount = 1, bool startRight = true);
  void moveJump();


private:
  int16_t forwardLimit = 80;
  int16_t backwardLimit = 280;
  uint16_t moveStep;
  uint32_t startTime;
  uint32_t lastMoveEndTime;
  uint32_t currentMoveStartTime;

  bool moveTimePassed(uint32_t time);
  // underscore because kP is a global variable :(
  double _kP;

  // 90 is forward, 180 is down, 270 is backwards
  int16_t angleRight;
  int16_t angleLeft;
  const int8_t NEUTRAL = 180;

  int16_t withinLimits(int16_t angle);
  void setLastMoveEndTime();
  void setCurrentMoveStartTime();

};



#endif