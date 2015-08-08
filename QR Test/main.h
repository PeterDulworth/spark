#include "application.h"

#define UUID_LENGTH 24

// qr scanner
#define QR_DELAY_AFTER_POWER_ON_MS 1000
#define QR_DELAY_AFTER_TRIGGER_MS 50
#define QR_COMMAND_PREFIX "\x02M\x0D"
#define QR_COMMAND_ACTIVATE "\x02T\x0D"
#define QR_COMMAND_DEACTIVATE "\x02U\x0D"
#define QR_READ_TIMEOUT 10000

// pin definitions
int pinLimitSwitch = A0;
int pinDeviceLED = A1;
int pinQRPower = A2;
int pinQRTrigger = A3;
int pinSensorLED = A4;
int pinSolenoid = A5;
int pinStepperSleep = D0;
int pinStepperDir = D1;
int pinStepperStep = D2;
int pinAssaySCL = D3;
int pinAssaySDA = D4;
int pinControlSCL = D5;
int pinControlSDA = D6;
//  D7 unassigned;

char qr_uuid[UUID_LENGTH + 1];
char particle_register[623];
