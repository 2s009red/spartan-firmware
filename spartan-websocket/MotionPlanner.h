#include "Arduino.h"

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H
typedef enum operating_mode {
  IDLING,
  SPARRING,
  COMBO,
} operating_mode_t;

struct Motion {
    uint32_t timestamp;
    uint32_t destination;
};

const size_t MAX_MOTION_SIZE = 100;
const uint32_t HOME_POSITION = 699;
const uint32_t WITHDRAW_DELAY = 200;

class MotionPlanner {
  public:
    MotionPlanner();

    void attachServo(uint8_t pin);
    void update();
    void clear();
    void clearAndReturnToZero();
    void addMotion(uint32_t timestamp, uint32_t destination);
    void startSparring(uint32_t spar_delay_minimum, uint32_t spar_delay_maximum, uint32_t extension); // TODO 
    void stopSparring();
    bool isSparring();

  private:
    Motion _motions[MAX_MOTION_SIZE];
    size_t _motionsLength;
    operating_mode_t _operatingMode;
    uint8_t _pin;

    uint32_t _spar_delay_minimum, _spar_delay_maximum, _extension;

    void _servoWrite(uint32_t position);
};

# endif   // MOTION_PLANNER_H
