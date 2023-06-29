#include "sequence.h"
#include <Arduino.h>

Sequence::Sequence()
{

}

void Sequence::update()
{

  if (currentSequence != nullptr)
    (this->*currentSequence)();

  if (currentMove != nullptr)
    (this->*currentMove)();

}

void Sequence::threeJumpSequence()
{

  if (moveCounter == 0){
    startMove(&Sequence::moveJump);
  }
  
  // how to ensure a move is "finished"? 
  // I can't make another timer or a flag, because there is a risk that I might forget to
  // include that at the end of a move... 

  // should start 2 seconds after jump
  if (sequenceTimePassed(8000) && moveCounter == 1){
    startMove(&Sequence::moveJump);
  }

  // should also start after 2 seconds
  if (sequenceTimePassed(9000) && moveTimePassed(8000) && moveCounter == 2){
    
    startMove(&Sequence::moveJump);
  }

  // should also start after 2 seconds
  if (sequenceTimePassed(24000)  && moveCounter == 3){
    
    startMove(&Sequence::moveStand); // cant use poseStand because it has arguments.
  }
  
}

void Sequence::startMove(void (Sequence::*move)()) {
  currentMove = move;
  currentMoveStartTime = millis();
  moveCounter ++;
}

void Sequence::startSequence(void (Sequence::*move)()) {
  currentSequence = move;
  currentSequenceStartTime = millis();
  moveCounter = 0;
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
  }
}

void Sequence::moveStand()
{
  poseStand(2);
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

double Sequence::getKP(){
  return _kP;
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

bool Sequence::sequenceTimePassed(uint32_t time){
  if ((currentSequenceStartTime + time) > millis()){
    return false;
  }
  return true;
}




void Sequence::moveWalk(uint8_t stepCount, bool startRight)
{

}


void Sequence::moveWalkSlow(uint8_t stepCount, bool startRight)
{

}