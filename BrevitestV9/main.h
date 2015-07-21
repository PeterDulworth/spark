#include "TCS34725.h"
#include "SoftI2CMaster.h"

// GLOBAL VARIABLES AND DEFINES

// general constants
#define FIRMWARE_VERSION 0x09
#define DATA_FORMAT_VERSION 0x01
#define UUID_LENGTH 24
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// eeprom
#define EEPROM_SERIAL_NUMBER_INDEX 8
#define EEPROM_SERIAL_NUMBER_LENGTH 19
#define EEPROM_ADDR_CALIBRATION_STEPS 16

// request
#define REQUEST_INDEX_INDEX UUID_LENGTH
#define REQUEST_INDEX_LENGTH 6
#define REQUEST_CODE_INDEX (REQUEST_INDEX_INDEX + REQUEST_INDEX_LENGTH)
#define REQUEST_CODE_LENGTH 2
#define REQUEST_PARAM_INDEX (REQUEST_CODE_INDEX + REQUEST_CODE_LENGTH)

// sensors
#define SENSOR_NUMBER_OF_SAMPLES 10

// assay
#define ASSAY_NAME_MAX_LENGTH 64

// params
#define PARAM_BYTES 18
#define PARAM_COUNT 10
#define PARAM_INDEX 28
#define PARAM_CHANGE_INDEX 2
#define PARAM_CHANGE_LENGTH 1
#define PARAM_CHANGE_VALUE 5

// caches
#define CACHE_COUNT_INDEX 2
#define ASSAY_CACHE_SIZE 2
#define ASSAY_CACHE_INDEX 70
#define ASSAY_RECORD_LENGTH 584
#define TEST_CACHE_SIZE 5
#define TEST_CACHE_INDEX 1238
#define TEST_RECORD_LENGTH 162
#define CACHE_SIZE_BYTES 2048

// BCODE
#define BCODE_CAPACITY 489
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)
#define BCODE_PAYLOAD_INDEX (BCODE_UUID_INDEX + UUID_LENGTH)

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(particle_status, STATUS_LENGTH, __VA_ARGS__)
#define TEST_DURATION_LENGTH 6

// device
#define PARTICLE_REGISTER_SIZE 622
#define PARTICLE_ARG_SIZE 63
#define PARTICLE_COMMAND_CODE_LENGTH 2
#define PARTICLE_COMMAND_PARAM_LENGTH 6

// QR scanner
#define QR_COMMAND_PREFIX "\x02M\x0D"
#define QR_COMMAND_ACTIVATE "\x02T\x0D"
#define QR_COMMAND_DEACTIVATE "\x02U\x0D"
#define QR_READ_TIMEOUT 5000

// pin definitions
int pinLimitSwitch = A0;
int pinSolenoid = A1;
int pinQRPower = A2;
//  A3 unassigned
int pinSensorLED = A4;
int pinDeviceLED = A5;
int pinStepperSleep = D0;
int pinStepperDir = D1;
int pinStepperStep = D2;
int pinAssaySDA = D3;
int pinAssaySCL = D4;
int pinControlSDA = D5;
int pinControlSCL = D6;
int pinQRTrigger = D7;

// global variables
bool device_busy;
bool test_in_progress;
bool start_test;
bool cancel_process;
unsigned long test_last_progress_update;

// BCODE globals
int BCODE_length;
int BCODE_count;
int BCODE_packets;
int BCODE_index;
char BCODE_uuid[UUID_LENGTH + 1];

// sensors
TCS34725 tcsAssay;
TCS34725 tcsControl;

// uuids
char request_uuid[UUID_LENGTH];
char user_uuid[UUID_LENGTH];
char cartridge_uuid[UUID_LENGTH];

// particle messaging
char particle_register[PARTICLE_REGISTER_SIZE + 1];
char particle_status[STATUS_LENGTH + 1];

struct Command {
    char arg[PARTICLE_ARG_SIZE + 1];
    int code;
    char param[PARTICLE_ARG_SIZE - PARTICLE_COMMAND_CODE_LENGTH + 1];
} particle_command;

struct Request {
    char arg[PARTICLE_ARG_SIZE + 1];
    char uuid[UUID_LENGTH];
    int code;
    int index;
    char param[PARTICLE_ARG_SIZE - UUID_LENGTH - REQUEST_CODE_LENGTH - REQUEST_INDEX_LENGTH + 1];
    Request() {
      arg[0] = '\0';
      uuid[0] = '\0';
      param[0] = '\0';
    }
} particle_request;

struct BrevitestSensorSampleRecord {
    int sample_time;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
};
BrevitestSensorSampleRecord assay_buffer[SENSOR_NUMBER_OF_SAMPLES];
BrevitestSensorSampleRecord control_buffer[SENSOR_NUMBER_OF_SAMPLES];

struct Param {
  uint16_t reset_steps;
  uint16_t step_delay_us;
  uint16_t wifi_ping_rate;
  uint16_t stepper_wake_delay_ms;
  uint8_t solenoid_surge_power;
  uint8_t solenoid_sustain_power;
  uint16_t solenoid_surge_period_ms;
  uint16_t delay_between_sensor_readings_ms;
  uint16_t sensor_params;
  uint16_t calibration_steps;
  uint16_t reserved[12];
  Param() {
    reset_steps = 14000;
    step_delay_us = 1200;
    wifi_ping_rate = 100;
    stepper_wake_delay_ms = 5;
    solenoid_surge_power = 255;
    solenoid_sustain_power = 200;
    solenoid_surge_period_ms = 300;
    delay_between_sensor_readings_ms = 500;
    sensor_params = (TCS34725_GAIN_4X << 8) + TCS34725_INTEGRATIONTIME_700MS;
    calibration_steps = 14000;
  }
};

struct BrevitestSensorRecord {
    int start_time;
    uint16_t red_norm;
    uint16_t green_norm;
    uint16_t blue_norm;
};

struct BrevitestTestRecord {
    int start_time;
    int finish_time;
    char test_uuid[UUID_LENGTH];
    char cartridge_uuid[UUID_LENGTH];
    char assay_uuid[UUID_LENGTH];
    Param param;
    BrevitestSensorRecord sensor_reading_initial_assay;
    BrevitestSensorRecord sensor_reading_initial_control;
    BrevitestSensorRecord sensor_reading_final_assay;
    BrevitestSensorRecord sensor_reading_final_control;
} *test_record;

struct BrevitestAssayRecord {
    char uuid[UUID_LENGTH];
    char name[ASSAY_NAME_MAX_LENGTH];
    int duration;
    uint8_t BCODE_length;
    uint8_t BCODE_version;
    char BCODE[BCODE_CAPACITY];
} *test_assay;

int test_index;
int assay_index;

struct EEPROM {
  uint8_t firmware_version;
  uint8_t data_format_version;
  uint8_t cache_count; // assay << 4 + test
  uint8_t reserved[5];
  char serial_number[EEPROM_SERIAL_NUMBER_LENGTH + 1];
  Param param;
  BrevitestAssayRecord assay_cache[ASSAY_CACHE_SIZE];
  BrevitestTestRecord test_cache[TEST_CACHE_SIZE];
} eeprom;
