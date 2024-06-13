#include "StepMotorController.h"

StepMotorController::StepMotorController(int dirPin1, int stepPin1, int enablePin1,
                                         int dirPin2, int stepPin2, int enablePin2,
                                         int stepDelay)
: _dirPin1(dirPin1), _stepPin1(stepPin1), _enablePin1(enablePin1),
  _dirPin2(dirPin2), _stepPin2(stepPin2), _enablePin2(enablePin2),
  _stepDelay(stepDelay), _moveEndTime(0), _moving(false), _stopped(true) {
}

void StepMotorController::begin() {
    pinMode(_dirPin1, OUTPUT);
    pinMode(_stepPin1, OUTPUT);
    pinMode(_enablePin1, OUTPUT);

    pinMode(_dirPin2, OUTPUT);
    pinMode(_stepPin2, OUTPUT);
    pinMode(_enablePin2, OUTPUT);

    digitalWrite(_enablePin1, LOW); // Motor 1'i etkinleştir
    digitalWrite(_enablePin2, LOW); // Motor 2'yi etkinleştir
}

void StepMotorController::move(int dirPin1State, int dirPin2State, unsigned long duration) {
    digitalWrite(_dirPin1, dirPin1State);
    digitalWrite(_dirPin2, dirPin2State);
    _moveEndTime = millis() + duration;
    _moving = true;
    _stopped = false;
    while (_moving) {
        update();
    }
}

void StepMotorController::ileri() {
    Serial.println("Motor ileri hareket ediyor...");
    move(LOW, HIGH, 1000);
}

void StepMotorController::geri() {
    Serial.println("Motor geri hareket ediyor...");
    move(HIGH, LOW, 1000);
}

void StepMotorController::sag() {
    Serial.println("Motor sağa dönüyor...");
    move(LOW, LOW, 500);
}

void StepMotorController::sol() {
    Serial.println("Motor sola dönüyor...");
    move(HIGH, HIGH, 500);
}

void StepMotorController::tara() {
    Serial.println("Tarama başladi...");
    move(HIGH, HIGH, 50);
}

void StepMotorController::dur() {
    if (!_stopped) {
        digitalWrite(_stepPin1, LOW);
        digitalWrite(_stepPin2, LOW);
        _moving = false;
        _stopped = true;
        Serial.println("Motor durdu.");
    }
}

void StepMotorController::update() {
    if (_moving && millis() < _moveEndTime) {
        stepMotor(_stepPin1);
        stepMotor(_stepPin2);
    } else {
        dur();
    }
}

void StepMotorController::stepMotor(int stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(_stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(_stepDelay);
}
