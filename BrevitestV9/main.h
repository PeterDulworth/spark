#include "TCS34725.h"
#include "SoftI2CMaster.h"
#include "flashee-eeprom.h"
    using namespace Flashee;

// GLOBAL VARIABLES AND DEFINES

// general constants
#define FIRMWARE_VERSION 0x09
#define FLASH_DATA_FORMAT_VERSION 0x01
#define SERIAL_NUMBER_LENGTH 19
#define UUID_LENGTH 24
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// request
#define REQUEST_INDEX_INDEX UUID_LENGTH
#define REQUEST_INDEX_LENGTH 6
#define REQUEST_CODE_INDEX (REQUEST_INDEX_INDEX + REQUEST_INDEX_LENGTH)
#define REQUEST_CODE_LENGTH 2
#define REQUEST_PARAM_INDEX (REQUEST_CODE_INDEX + REQUEST_CODE_LENGTH)

#define SENSOR_NUMBER_OF_SAMPLES 10

// params
#define PARAM_CODE_LENGTH 3
#define PARAM_COUNT 9
#define PARAM_TOTAL_LENGTH 40

// caches
#define ASSAY_CACHE_SIZE 2
#define TEST_CACHE_SIZE 5
#define CACHE_SIZE_BYTES 2048

// BCODE
#define BCODE_CAPACITY 489
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)
#define BCODE_PAYLOAD_INDEX (BCODE_UUID_INDEX + UUID_LENGTH)

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(spark_status, STATUS_LENGTH, __VA_ARGS__)

// device
#define PARTICLE_REGISTER_SIZE 622
#define PARTICLE_ARG_SIZE 63
#define PARTICLE_COMMAND_CODE_LENGTH 2

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
bool device_ready;
bool init_device;
bool start_test;
bool test_in_progress;
bool collect_sensor_data;
bool cancel_process;

// BCODE globals
int BCODE_length;
int BCODE_count;
int BCODE_packets;
int BCODE_index;
char BCODE_uuid[UUID_LENGTH + 1];

// test
char test_uuid[UUID_LENGTH + 1];
unsigned long test_last_progress_update;
uint8_t test_sensor_sample_count;
uint8_t test_sensor_reading_count;

// sensors
TCS34725 tcsAssay;
TCS34725 tcsControl;

// spark messaging
char spark_register[SPARK_REGISTER_SIZE + 1];
char spark_status[STATUS_LENGTH + 1];

struct Command {
    char arg[PARTICLE_ARG_SIZE + 1];
    int code;
    char param[PARTICLE_ARG_SIZE - COMMAND_CODE_LENGTH + 1];
} spark_command;

struct Request {
    char arg[SPARK_ARG_SIZE + 1];
    char uuid[UUID_LENGTH];
    int code;
    int index;
    char param[SPARK_ARG_SIZE - UUID_LENGTH - REQUEST_CODE_LENGTH - REQUEST_INDEX_LENGTH + 1];
    Request() {
      arg[0] = '\0';
      uuid[0] = '\0';
      param[0] = '\0';
    }
} spark_request;

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
  uint16_t solenoid_surge_power;
  uint16_t solenoid_surge_period_ms;
  uint16_t solenoid_sustain_power;
  uint16_t delay_between_sensor_readings_ms;
  uint16_t sensor_params;
  uint16_t reserved[12];
  Param() {
    reset_steps = 14000;
    step_delay_us = 1200;
    wifi_ping_rate = 100;
    stepper_wake_delay_ms = 5;
    solenoid_surge_power = 255;
    solenoid_surge_period_ms = 300;
    solenoid_sustain_power = 200;
    delay_between_sensor_readings_ms = 500;
    sensor_params = (TCS34725_GAIN_4X << 8) + TCS34725_INTEGRATIONTIME_700MS;
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
} test_record;

struct BrevitestAssayRecord {
    char id[UUID_LENGTH];
    char name[ASSAY_NAME_MAX_LENGTH];
    uint8_t BCODE_length;
    uint8_t BCODE_version;
    char BCODE[BCODE_CAPACITY];
} test_assay;

struct EEPROM {
  uint8_t firmware_version;
  uint8_t data_format_version;
  char reserved[14];
  char serial_number[SERIAL_NUMBER_LENGTH + 1];
  Param default_param;
  BrevitestAssayRecord assay_cache[ASSAY_CACHE_SIZE];
  BrevitestTestRecord test_cache[TEST_CACHE_SIZE];
} eeprom;
