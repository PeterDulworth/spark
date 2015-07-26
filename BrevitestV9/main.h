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
#define EEPROM_ADDR_FIRMWARE_VERSION 0
#define EEPROM_ADDR_DATA_FORMAT_VERSION 1
#define EEPROM_ADDR_CACHE_COUNT 2
#define EEPROM_ADDR_SERIAL_NUMBER 8
#define EEPROM_SERIAL_NUMBER_LENGTH 19
#define EEPROM_ADDR_PARAM 28
#define EEPROM_PARAM_LENGTH 18
#define EEPROM_PARAM_COUNT 10
#define EEPROM_OFFSET_PARAM_RESET_STEPS 0
#define EEPROM_OFFSET_PARAM_STEP_DELAY_US 2
#define EEPROM_OFFSET_PARAM_WIFI_PING_RATE 4
#define EEPROM_OFFSET_PARAM_STEPPER_WAKE_DELAY_MS 6
#define EEPROM_OFFSET_PARAM_SOLENOID_SURGE_POWER 8
#define EEPROM_OFFSET_PARAM_SOLENOID_SUSTAIN_POWER 9
#define EEPROM_OFFSET_PARAM_SOLENOID_SURGE_PERIOD_MS 10
#define EEPROM_OFFSET_PARAM_DELAY_BETWEEN_SAMPLES 12
#define EEPROM_OFFSET_PARAM_SENSOR_PARAMS 14
#define EEPROM_OFFSET_PARAM_CALIBRATION_STEPS 16
#define EEPROM_ADDR_ASSAY_CACHE 70
#define EEPROM_ADDR_TEST_CACHE 1238

// sensors
#define SENSOR_NUMBER_OF_SAMPLES 10

// assay
#define ASSAY_NAME_MAX_LENGTH 64
#define ASSAY_PACKETS_INDEX 0
#define ASSAY_PACKETS_LENGTH 2
#define ASSAY_UUID_INDEX (ASSAY_PACKETS_INDEX + ASSAY_PACKETS_LENGTH)
#define ASSAY_BCODE_CAPACITY 489

// buffer mappings
#define ASSAY_BUFFER_UUID_INDEX 0
#define ASSAY_BUFFER_NAME_INDEX (ASSAY_BUFFER_UUID_INDEX + UUID_LENGTH)
#define ASSAY_BUFFER_DURATION_INDEX (ASSAY_BUFFER_NAME_INDEX + ASSAY_NAME_MAX_LENGTH)
#define ASSAY_BUFFER_DURATION_LENGTH 6
#define ASSAY_BUFFER_BCODE_SIZE_INDEX (ASSAY_BUFFER_DURATION_INDEX + ASSAY_BUFFER_DURATION_LENGTH)
#define ASSAY_BUFFER_BCODE_SIZE_LENGTH 3
#define ASSAY_BUFFER_BCODE_VERSION_INDEX (ASSAY_BUFFER_BCODE_SIZE_INDEX + ASSAY_BUFFER_BCODE_SIZE_LENGTH)
#define ASSAY_BUFFER_BCODE_VERSION_LENGTH 3
#define ASSAY_BUFFER_BCODE_INDEX (ASSAY_BUFFER_BCODE_VERSION_INDEX + ASSAY_BUFFER_BCODE_VERSION_LENGTH)

#define TEST_BUFFER_TEST_UUID_INDEX 0
#define TEST_BUFFER_CARTRIDGE_UUID_INDEX (TEST_BUFFER_TEST_UUID_INDEX + UUID_LENGTH)
#define TEST_BUFFER_ASSAY_UUID_INDEX (TEST_BUFFER_CARTRIDGE_UUID_INDEX + UUID_LENGTH)

// params
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
#define BUFFER_MESSAGE_SIZE_INDEX BUFFER_PAYLOAD_INDEX
#define BUFFER_MESSAGE_SIZE_LENGTH 3
#define BUFFER_NUMBER_OF_PACKETS_INDEX (BUFFER_MESSAGE_SIZE_INDEX + BUFFER_MESSAGE_SIZE_LENGTH)
#define BUFFER_NUMBER_OF_PACKETS_LENGTH 2

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(particle_status, STATUS_LENGTH, __VA_ARGS__)
#define TEST_DURATION_LENGTH 6

// device LED
#define DEVICE_LED_BLINK_DELAY 500

// qr scanner
#define QR_DELAY_AFTER_POWER_ON_MS 1000
#define QR_DELAY_AFTER_TRIGGER_MS 50
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

// device LED
struct DeviceLED {
    bool blinking;
    bool currently_on;
    unsigned long blink_change_time;
    unsigned long blink_timeout;
} device_LED;

// buffer
struct BrevitestBuffer {
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
char request_uuid[UUID_LENGTH];
char claimant_uuid[UUID_LENGTH];
char test_uuid[UUID_LENGTH];
char qr_uuid[UUID_LENGTH];

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
    char BCODE[ASSAY_BCODE_CAPACITY];
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
