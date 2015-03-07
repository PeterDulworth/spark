#include "TCS34725.h"
#include "SoftI2CMaster.h"
#include "flashee-eeprom.h"
    using namespace Flashee;

// GLOBAL VARIABLES AND DEFINES

// general constants
#define FIRMWARE_VERSION 0x08
#define SERIAL_NUMBER_LENGTH 19
#define UUID_LENGTH 32
#define COMMAND_CODE_LENGTH 2
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// request
#define REQUEST_INDEX_INDEX UUID_LENGTH
#define REQUEST_INDEX_LENGTH 6
#define REQUEST_CODE_INDEX (REQUEST_INDEX_INDEX + REQUEST_INDEX_LENGTH)
#define REQUEST_CODE_LENGTH 2
#define REQUEST_PARAM_INDEX (REQUEST_CODE_INDEX + REQUEST_CODE_LENGTH)

// flash
FlashDevice* flash;
#define FLASH_RECORD_CAPACITY 600
#define FLASH_OVERFLOW_ADDRESS 450516

// test
#define TEST_NUMBER_OF_RECORDS_ADDR 512
#define TEST_RECORD_LENGTH 900
#define TEST_RECORD_START_ADDR 516
#define TEST_RECORD_UUID_OFFSET 6
#define TEST_RECORD_PARAM_OFFSET 38
#define TEST_RECORD_BCODE_OFFSET 106
#define TEST_RECORD_SENSOR_OFFSET 600
#define TEST_RECORD_STRING_LENGTH 1300
#define TEST_RECORD_NUM_LENGTH 3
#define TEST_RECORD_INDEX_LENGTH 6

// sensor
#define SENSOR_COLLECT_SAMPLES 10
#define SENSOR_SAMPLE_LENGTH 15
#define SENSOR_SAMPLE_CAPACITY 20

// eeprom addresses
#define EEPROM_ADDR_DEFAULT_FLAG 0
#define EEPROM_ADDR_SERIAL_NUMBER 1
#define EEPROM_ADDR_CALIBRATION_STEPS 20

// params
#define PARAM_CODE_LENGTH 3
#define PARAM_CAPACITY 16
#define PARAM_COUNT 10
#define PARAM_TOTAL_LENGTH (PARAM_COUNT * 4)

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(spark_status, STATUS_LENGTH, __VA_ARGS__)

// device
#define SPARK_REGISTER_SIZE 622
#define SPARK_ARG_SIZE 63
#define SPARK_RESET_STAGE_STEPS -60000

// BCODE
#define BCODE_CAPACITY 494
#define BCODE_PAYLOAD_INDEX 37
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)

// pin definitions
int pinSolenoid = A1;
int pinStepperStep = D2;
int pinStepperDir = D1;
int pinStepperSleep = D0;
int pinLimitSwitch = A0;
int pinSensorLED = A4;
int pinDeviceLED = A5;
int pinAssaySDA = D3;
int pinAssaySCL = D4;
int pinControlSDA = D5;
int pinControlSCL = D6;

// global variables
bool device_ready;
bool init_device;
bool run_assay;
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
int test_start_time;
int test_num;
char test_string[TEST_RECORD_STRING_LENGTH];
int test_string_length;
char test_sensor_sample_count;

// sensors
TCS34725 tcsAssay;
TCS34725 tcsControl;

// spark messaging
char spark_register[SPARK_REGISTER_SIZE + 1];
char spark_status[STATUS_LENGTH + 1];

struct Command {
    char arg[SPARK_ARG_SIZE + 1];
    int code;
    char param[SPARK_ARG_SIZE - COMMAND_CODE_LENGTH + 1];
} spark_command;

struct Request {
    char arg[SPARK_ARG_SIZE + 1];
    char uuid[UUID_LENGTH + 1];
    int code;
    int index;
    char param[SPARK_ARG_SIZE - UUID_LENGTH - REQUEST_CODE_LENGTH - REQUEST_INDEX_LENGTH + 1];
    Request() {
        uuid[0] = '\0';
    }
} spark_request;

struct Param {
    // stepper
    int step_delay_us;  // microseconds
    int stepper_wifi_ping_rate;
    int stepper_wake_delay_ms; // milliseconds

    // solenoid
    int solenoid_surge_power;
    int solenoid_surge_period_ms; // milliseconds
    int solenoid_sustain_power;

    // sensors
    int sensor_params;
    int sensor_ms_between_samples;
    int sensor_led_power;
    int sensor_led_warmup_ms;

    Param() {  //  DEFAULT VALUES
        // stepper
        step_delay_us = 1200;  // microseconds
        stepper_wifi_ping_rate = 100;
        stepper_wake_delay_ms = 5; // milliseconds

        // solenoid
        solenoid_surge_power = 255;
        solenoid_surge_period_ms = 200; // milliseconds
        solenoid_sustain_power = 100;

        // sensors
        sensor_params = (TCS34725_INTEGRATIONTIME_50MS << 8) + TCS34725_GAIN_4X;
        sensor_ms_between_samples = 1000;
        sensor_led_power = 20;
        sensor_led_warmup_ms = 10000;
    }
} brevitest;

struct BrevitestSensorRecord {
    char sensor_code;
    uint8_t sample_number;
    int reading_time;
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

struct BrevitestTestRecord{
    uint16_t num;
    int start_time;
    char uuid[UUID_LENGTH + 1];
    uint8_t num_samples;  // 2 readings per sample (assay, control)
    uint8_t BCODE_version;
    uint16_t BCODE_length;
    Param param;
    char BCODE[BCODE_CAPACITY];
    BrevitestSensorRecord sensor_reading[2 * SENSOR_SAMPLE_CAPACITY];
} test_record;
