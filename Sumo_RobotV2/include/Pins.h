#pragma once
#include <Arduino.h>

namespace Pins {
  // IR + LED
  constexpr uint8_t IR_RX_PIN        = 4;   // IR receive pin
  constexpr uint8_t LED_PIN          = 11;  // status LED

  // QTR Line Sensors
  constexpr uint8_t QTR_LEFT_PIN     = 2;
  constexpr uint8_t QTR_RIGHT_PIN    = 3;
/*   constexpr uint8_t QTR_LEFT_PIN     = A0;
  constexpr uint8_t QTR_RIGHT_PIN    = A1; */
/*   // Left Motor
  constexpr uint8_t LEFT_MOTOR_IN1_PIN  = 7;
  constexpr uint8_t LEFT_MOTOR_IN2_PIN  = 8;
  constexpr uint8_t LEFT_MOTOR_PWM_PIN  = 5;

  // Right Motor
  constexpr uint8_t RIGHT_MOTOR_IN1_PIN = 9;
  constexpr uint8_t RIGHT_MOTOR_IN2_PIN = 10;
  constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 6; */
  // Left Motor
  constexpr uint8_t LEFT_MOTOR_IN1_PIN  = 7;
  constexpr uint8_t LEFT_MOTOR_IN2_PIN  = 8;
  constexpr uint8_t LEFT_MOTOR_PWM_PIN  = 5;

  // Right Motor
  constexpr uint8_t RIGHT_MOTOR_IN1_PIN = 9;
  constexpr uint8_t RIGHT_MOTOR_IN2_PIN = 10;
  constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 6;

  // Ultrasonic Sensor
  constexpr uint8_t PIN_TRIG            = 12;
  constexpr uint8_t PIN_ECHO_LEFT       = A0;
  constexpr uint8_t PIN_ECHO_FRONT      = A1;
  constexpr uint8_t PIN_ECHO_RIGHT      = A2;

  // VL53L0X XSHUT pins
  constexpr uint8_t TOF_XSHUT_RIGHT_PIN = A0;
  constexpr uint8_t TOF_XSHUT_FRONT_PIN = A1;
  constexpr uint8_t TOF_XSHUT_LEFT_PIN  = A2;

/*   constexpr uint8_t TOF_XSHUT_RIGHT_PIN = A2;
  constexpr uint8_t TOF_XSHUT_FRONT_PIN = A3;
  constexpr uint8_t TOF_XSHUT_LEFT_PIN  = 12; */
} // namespace Pins