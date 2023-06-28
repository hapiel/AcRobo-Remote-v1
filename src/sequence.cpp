#include "sequence.h"
#include <Arduino.h>

Sequence::Sequence()
{

}

void Sequence::update()
{
  // update timer
  // check for new moves to execute
  // execute within limits
}

void Sequence::testSequence()
{
  setCurrentMoveStartTime(); 
  moveJump(); // How to make sure moveJump gets called in every update() for the duration of the move?


  
}

void Sequence::setLastMoveEndTime()
{
  lastMoveEndTime = millis(); 
}

void Sequence::setCurrentMoveStartTime()
{
  currentMoveStartTime = millis(); 
}


int16_t Sequence::withinLimits(int16_t angle)
{
  return min(max(angle, forwardLimit),backwardLimit);
}


// main loop needs to get these positions
uint16_t Sequence::getLPos(){
  return withinLimits(angleLeft);
}

uint16_t Sequence::getRPos(){
  return withinLimits(angleRight);
}


void Sequence::poseStand(double _kP)
{
  poseBow(3, _kP);
}
void Sequence::poseBow(int16_t angle, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL - angle;
  angleRight = NEUTRAL - angle;
}
void Sequence::poseKickL (int16_t angle, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL - angle;
  angleRight = NEUTRAL;
}
void Sequence::poseKickR (int16_t angle, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL;
  angleRight = NEUTRAL - angle;
}
void Sequence::poseStepL (int16_t angle, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL - angle;
  angleRight = NEUTRAL + angle;
}
void Sequence::poseStepR (int16_t angle, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL + angle;
  angleRight = NEUTRAL - angle;
}
void Sequence::poseCustom(int16_t angleL, int16_t angleR, double _kP)
{
  this->_kP = _kP;
  angleLeft = NEUTRAL + angleL;
  angleRight = NEUTRAL + angleR;
}

bool Sequence::moveTimePassed(uint32_t time){
  if ((currentMoveStartTime + time) > millis()){
    return false;
  }
  return true;

}

void Sequence::moveJump()
{
  poseStand();

  if (moveTimePassed(2000)){
    poseBow(45, 0.6);
  }

  if (moveTimePassed(3000)){
    poseBow(-10, 4);
  }

  if (moveTimePassed(3800)){
    poseBow(10, 2);
  }
  if (moveTimePassed(6000)){
    poseStand();
    setLastMoveEndTime();
  }
}



void Sequence::moveWalk(uint8_t stepCount, bool startRight)
{

}


void Sequence::moveWalkSlow(uint8_t stepCount, bool startRight)
{

}