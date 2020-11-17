#include "MotionPlanner.h"

MotionPlanner::MotionPlanner() {
    _motionsLength = 0;
    _operatingMode = IDLING;
}

void MotionPlanner::attachServo(uint8_t pin) {
    _pin = pin;
}

void MotionPlanner::update() {
    if (!_motionsLength) return;

    uint32_t milliseconds = millis();

    if (_operatingMode == SPARRING && milliseconds > _motions[1].timestamp) {
        clear();

        uint32_t nextPunchTime = random(_spar_delay_minimum, _spar_delay_maximum);
        addMotion(milliseconds + nextPunchTime, random(90, _extension));  // FIXME
        addMotion(milliseconds + nextPunchTime + WITHDRAW_DELAY, 0);
        _servoWrite(0);
    }
    
    for (size_t i = _motionsLength - 1; i >= 0; i--) {
        if (milliseconds > _motions[i].timestamp) {
            _servoWrite(_motions[i].destination);
            return;
        }
    }

    _servoWrite(0);
    return;
}

void MotionPlanner::_servoWrite(uint32_t _position) {
    uint32_t pwm_value = map(_position, 0, 180, 699, 504);
    analogWrite(_pin, pwm_value);

    return;
}

void MotionPlanner::clear() {
    _motionsLength = 0;

    return;
}

void MotionPlanner::clearAndReturnToZero() {
    _motionsLength = 0;
    _servoWrite(0);

    return;
}

void MotionPlanner::addMotion(uint32_t timestamp, uint32_t destination) {
    _motions[_motionsLength].timestamp = timestamp;
    _motions[_motionsLength].destination = destination;

    _motionsLength++;

    return;
}

bool MotionPlanner::isSparring() {
    return _operatingMode == SPARRING;
}

void MotionPlanner::startSparring(uint32_t spar_delay_minimum, uint32_t spar_delay_maximum, uint32_t extension) {
    Serial.println(spar_delay_minimum);
    Serial.println(spar_delay_maximum);
    _spar_delay_minimum = spar_delay_minimum;
    _spar_delay_maximum = spar_delay_maximum;
    _extension = extension;

    _operatingMode = SPARRING;

    uint32_t nextPunchTime = random(_spar_delay_minimum, _spar_delay_maximum);
    addMotion(millis() + nextPunchTime, random(90, _extension));  // FIXME
    addMotion(millis() + nextPunchTime + WITHDRAW_DELAY, 0);
}

void MotionPlanner::stopSparring() {
    clear();
    _operatingMode = IDLING;
}
