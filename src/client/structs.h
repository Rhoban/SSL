#pragma once

#include <stdint.h>

/**
 * WARNING: This not should be edited without re-synchronizing with the struct
 * in the mainboard firmware (see SSL-Electronics/mainboard/firmware)
 */

 struct packet_master {
     #define ACTION_ON      (1<<0)   // The robot should be on (else everything is stopped)
     #define ACTION_KICK1   (1<<1)   // Kick on kicker 1 (transition from 0 to 1 trigger kick)
     #define ACTION_KICK2   (1<<2)   // Kick on kicker 2 (transition from 0 to 1 trigger kick)
     #define ACTION_DRIBBLE (1<<3)   // Enable/disable the dribbler
     #define ACTION_CHARGE  (1<<4)   // Enable/disable the capacitor charge
     uint8_t actions;

     float wheel1;                   // Wheel speeds in turn/s
     float wheel2;
     float wheel4;
     float wheel3;

     uint16_t kickPower;             // Kick power (this is a duration in uS)
 } __attribute__((packed));

 struct packet_robot {
     uint8_t id;

     #define STATUS_OK           (1<<0)  // The robot is alive and ok
     #define STATUS_DRIVER_ERR   (1<<1)  // Error with drivers
     #define STATUS_IR           (1<<2)  // The infrared barrier detects something
     uint8_t status;

     uint16_t cap_volt;                  // Kick capcaitor voltage (10th of V)

     uint8_t bat1_volt;                  // Battery 1 voltage (10th of V)
     uint8_t bat2_volt;                  // Battery 2 voltage (10th of V)
 } __attribute__((packed));
