#include "TCS34725.h"
#include "SoftI2CMaster.h"

// GLOBAL VARIABLES AND DEFINES

// general constants
#define FIRMWARE_VERSION 9
#define DATA_FORMAT_VERSION 1
#define UUID_LENGTH 24
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// cartridge ids for testing

#define CARTRIDGE_TEST_ID_LAPTOP "555cc19af0fbbdd210de6e48"
#define CARTRIDGE_TEST_ID_DESKTOP "557c907edf2705463b0d7819"
#define CARTRIDGE_TEST_ID_CLOUD "55be83d61bbc552671bb0de3"

// eeprom
#define EEPROM_ADDR_FIRMWARE_VERSION offsetof(Particle_EEPROM, firmware_version)
#define EEPROM_ADDR_DATA_FORMAT_VERSION offsetof(Particle_EEPROM, data_format_version)
#define EEPROM_ADDR_CACHE_COUNT offsetof(Particle_EEPROM, cache_count)
#define EEPROM_ADDR_SERIAL_NUMBER offsetof(Particle_EEPROM, serial_number)
#define EEPROM_SERIAL_NUMBER_LENGTH 19
#define EEPROM_ADDR_PARAM offsetof(Particle_EEPROM, param)
#define EEPROM_PARAM_LENGTH 18
#define EEPROM_PARAM_COUNT 10
#define EEPROM_OFFSET_PARAM_RESET_STEPS offsetof(Particle_EEPROM, param.reset_steps)
#define EEPROM_OFFSET_PARAM_STEP_DELAY_US offsetof(Particle_EEPROM, param.step_delay_us)
#define EEPROM_OFFSET_PARAM_PUBLISH_INTERVAL_DURING_MOVE offsetof(Particle_EEPROM, param.publish_interval_during_move)
#define EEPROM_OFFSET_PARAM_STEPPER_WAKE_DELAY_MS offsetof(Particle_EEPROM, param.stepper_wake_delay_ms)
#define EEPROM_OFFSET_PARAM_SOLENOID_SURGE_POWER offsetof(Particle_EEPROM, param.solenoid_surge_power)
#define EEPROM_OFFSET_PARAM_SOLENOID_SUSTAIN_POWER offsetof(Particle_EEPROM, param.solenoid_sustain_power)
#define EEPROM_OFFSET_PARAM_SOLENOID_SURGE_PERIOD_MS offsetof(Particle_EEPROM, param.solenoid_surge_period_ms)
#define EEPROM_OFFSET_PARAM_DELAY_BETWEEN_SAMPLES offsetof(Particle_EEPROM, param.delay_between_sensor_readings_ms)
#define EEPROM_OFFSET_PARAM_SENSOR_PARAMS offsetof(Particle_EEPROM, param.sensor_params)
#define EEPROM_OFFSET_PARAM_CALIBRATION_STEPS offsetof(Particle_EEPROM, param.calibration_steps)
#define EEPROM_ADDR_ASSAY_CACHE offsetof(Particle_EEPROM, assay_cache)
#define EEPROM_ADDR_TEST_CACHE offsetof(Particle_EEPROM, test_cache)

// sensors
#define SENSOR_NUMBER_OF_SAMPLES 10

// assay
#define ASSAY_NAME_MAX_LENGTH 63
#define ASSAY_BCODE_CAPACITY 489

// buffer mappings
#define ASSAY_BUFFER_UUID_INDEX 0
#define ASSAY_BUFFER_NAME_LENGTH_INDEX (ASSAY_BUFFER_UUID_INDEX + UUID_LENGTH)
#define ASSAY_BUFFER_NAME_LENGTH_LENGTH 2
#define ASSAY_BUFFER_NAME_INDEX (ASSAY_BUFFER_NAME_LENGTH_INDEX + ASSAY_BUFFER_NAME_LENGTH_LENGTH)
#define ASSAY_BUFFER_DURATION_LENGTH 5
#define ASSAY_BUFFER_BCODE_SIZE_LENGTH 3
#define ASSAY_BUFFER_BCODE_VERSION_LENGTH 3

#define TEST_BUFFER_START_TIME_INDEX 0
#define TEST_BUFFER_START_TIME_LENGTH 11
#define TEST_BUFFER_FINISH_TIME_INDEX (TEST_BUFFER_START_TIME_INDEX + TEST_BUFFER_START_TIME_LENGTH)
#define TEST_BUFFER_FINISH_TIME_LENGTH 11
#define TEST_BUFFER_TEST_UUID_INDEX (TEST_BUFFER_FINISH_TIME_INDEX + TEST_BUFFER_FINISH_TIME_LENGTH)
#define TEST_BUFFER_CARTRIDGE_UUID_INDEX (TEST_BUFFER_TEST_UUID_INDEX + UUID_LENGTH)
#define TEST_BUFFER_ASSAY_UUID_INDEX (TEST_BUFFER_CARTRIDGE_UUID_INDEX + UUID_LENGTH)
#define TEST_BUFFER_PARAM_INDEX (TEST_BUFFER_ASSAY_UUID_INDEX + UUID_LENGTH)
#define TEST_BUFFER_PARAM_LENGTH sizeof(Param)
#define TEST_BUFFER_INITIAL_ASSAY_SENSOR_INDEX (TEST_BUFFER_PARAM_INDEX + TEST_BUFFER_PARAM_LENGTH)
#define TEST_BUFFER_INITIAL_CONTROL_SENSOR_INDEX (TEST_BUFFER_INITIAL_ASSAY_SENSOR_INDEX + SENSOR_RECORD_LENGTH)
#define TEST_BUFFER_FINAL_ASSAY_SENSOR_INDEX (TEST_BUFFER_INITIAL_CONTROL_SENSOR_INDEX + SENSOR_RECORD_LENGTH)
#define TEST_BUFFER_FINAL_CONTROL_SENSOR_INDEX (TEST_BUFFER_FINAL_ASSAY_SENSOR_INDEX + SENSOR_RECORD_LENGTH)

// params
#define PARAM_CHANGE_INDEX 2
#define PARAM_CHANGE_LENGTH 1
#define PARAM_CHANGE_VALUE 5

// caches
#define CACHE_COUNT_INDEX 2
#define ASSAY_CACHE_SIZE 2
#define ASSAY_CACHE_INDEX offsetof(Particle_EEPROM, assay_cache)
#define ASSAY_RECORD_LENGTH sizeof(BrevitestAssayRecord)
#define TEST_CACHE_SIZE 4
#define TEST_CACHE_INDEX offsetof(Particle_EEPROM, test_cache)
#define TEST_RECORD_LENGTH sizeof(BrevitestTestRecord)
#define SENSOR_RECORD_LENGTH sizeof(BrevitestSensorRecord)
#define CACHE_SIZE_BYTES sizeof(Particle_EEPROM)

// particle
#define PARTICLE_REGISTER_SIZE 622
#define PARTICLE_ARG_SIZE 63
#define PARTICLE_COMMAND_CODE_INDEX 0
#define PARTICLE_COMMAND_CODE_LENGTH 2
#define PARTICLE_COMMAND_PARAM_INDEX (PARTICLE_COMMAND_CODE_INDEX + PARTICLE_COMMAND_CODE_LENGTH)
#define PARTICLE_COMMAND_PARAM_LENGTH 6
#define PARTICLE_REQUEST_CODE_INDEX 0
#define PARTICLE_REQUEST_CODE_LENGTH 2
#define PARTICLE_REQUEST_PARAM_INDEX (PARTICLE_REQUEST_CODE_INDEX + PARTICLE_REQUEST_CODE_LENGTH)
#define PARTICLE_PUBLISH_INTERVAL 1000

// buffer
#define BUFFER_SIZE 600
#define BUFFER_PACKET_NUMBER_INDEX 0
#define BUFFER_PACKET_NUMBER_LENGTH 2
#define BUFFER_PAYLOAD_SIZE_INDEX (BUFFER_PACKET_NUMBER_INDEX + BUFFER_PACKET_NUMBER_LENGTH)
#define BUFFER_PAYLOAD_SIZE_LENGTH 2
#define BUFFER_ID_INDEX (BUFFER_PAYLOAD_SIZE_INDEX + BUFFER_PAYLOAD_SIZE_LENGTH)
#define BUFFER_ID_LENGTH 6
#define BUFFER_PAYLOAD_INDEX (BUFFER_ID_INDEX + BUFFER_ID_LENGTH)
#define BUFFER_PAYLOAD_MAX_LENGTH (PARTICLE_ARG_SIZE - PARTICLE_COMMAND_CODE_LENGTH - BUFFER_PAYLOAD_INDEX)
// first packet payload
#define BUFFER_MESSAGE_TYPE_INDEX BUFFER_PAYLOAD_INDEX
#define BUFFER_MESSAGE_TYPE_LENGTH 2
#define BUFFER_MESSAGE_SIZE_INDEX (BUFFER_MESSAGE_TYPE_INDEX + BUFFER_MESSAGE_TYPE_LENGTH)
#define BUFFER_MESSAGE_SIZE_LENGTH 3
#define BUFFER_NUMBER_OF_PACKETS_INDEX (BUFFER_MESSAGE_SIZE_INDEX + BUFFER_MESSAGE_SIZE_LENGTH)
#define BUFFER_NUMBER_OF_PACKETS_LENGTH 2

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(particle_status, STATUS_LENGTH, __VA_ARGS__)
#define TEST_DURATION_LENGTH 6

// device LED
#define DEVICE_LED_BLINK_DELAY_DEFAULT 500
#define DEVICE_LED_BLINK_DELAY_DEFAULTCLAIMED_DELAY 100
#define DEVICE_LED_BLINK_NO_TIMEOUT 0

// qr scanner
#define QR_DELAY_AFTER_POWER_ON_MS 1000
#define QR_DELAY_AFTER_TRIGGER_MS 50
#define QR_COMMAND_PREFIX "\x02M\x0D"
#define QR_COMMAND_ACTIVATE "\x02T\x0D"
#define QR_COMMAND_DEACTIVATE "\x02U\x0D"
#define QR_READ_TIMEOUT 20000

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

// global variables
bool test_in_progress;
bool start_test;
bool cancel_process;
bool calibrate;

// device LED
struct DeviceLED {
    bool blinking;
    bool currently_on;
    unsigned long blink_rate;
    unsigned long blink_change_time;
    unsigned long blink_timeout;
    DeviceLED() {
      blinking = false;
      currently_on = false;
      blink_rate = DEVICE_LED_BLINK_DELAY_DEFAULT;
      blink_change_time = 0;
      blink_timeout = 0;
    }
} device_LED;

// buffer
struct BrevitestBuffer {
  int message_type;
  int packet_number;
  int payload_size;
  int index;
  char id[BUFFER_ID_LENGTH];
  int message_size;
  int number_of_packets;
  char buffer[BUFFER_SIZE];
} data_transfer;

// sensors
TCS34725 tcsAssay;
TCS34725 tcsControl;

// progress
int test_progress;
int test_percent_complete;
unsigned long test_last_progress_update;

// uuids
char user_uuid[UUID_LENGTH + 1];
char claimant_uuid[UUID_LENGTH + 1];
char test_uuid[UUID_LENGTH + 1];
char qr_uuid[UUID_LENGTH + 1];

// particle messaging
char particle_register[PARTICLE_REGISTER_SIZE + 1];
char particle_status[STATUS_LENGTH + 1];
int transfer_type;

struct Command {
    char arg[PARTICLE_ARG_SIZE + 1];
    int code;
    char param[PARTICLE_ARG_SIZE - PARTICLE_COMMAND_CODE_LENGTH + 1];
} particle_command;

struct Request {
    char arg[PARTICLE_ARG_SIZE + 1];
    int code;
    char param[PARTICLE_ARG_SIZE - PARTICLE_REQUEST_CODE_LENGTH + 1];
    Request() {
      arg[0] = '\0';
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
  uint16_t publish_interval_during_move;
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
    publish_interval_during_move = 100;
    stepper_wake_delay_ms = 5;
    solenoid_surge_power = 255;
    solenoid_sustain_power = 200;
    solenoid_surge_period_ms = 300;
    delay_between_sensor_readings_ms = 500;
    sensor_params = (TCS34725_GAIN_4X << 8) + TCS34725_INTEGRATIONTIME_700MS;
    calibration_steps = 250;
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
    uint8_t name_length;
    char name[ASSAY_NAME_MAX_LENGTH];
    int duration;
    uint8_t BCODE_length;
    uint8_t BCODE_version;
    char BCODE[ASSAY_BCODE_CAPACITY];
} *test_assay;

int test_index;
int assay_index;

struct Particle_EEPROM {
  uint8_t firmware_version;
  uint8_t data_format_version;
  uint8_t cache_count; // assay << 4 + test
  uint8_t reserved1;
  uint8_t reserved2[4];
  char serial_number[EEPROM_SERIAL_NUMBER_LENGTH + 1]; // includes trailing \0
  Param param;
  BrevitestAssayRecord assay_cache[ASSAY_CACHE_SIZE];
  BrevitestTestRecord test_cache[TEST_CACHE_SIZE];
} eeprom;
