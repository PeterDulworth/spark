#include "TCS34725.h"
#include "SoftI2CMaster.h"
#include "flashee-eeprom.h"
    using namespace Flashee;

// GLOBAL VARIABLES AND DEFINES

// general constants
#define FIRMWARE_VERSION 0x08
#define SERIAL_NUMBER_LENGTH 19
#define UUID_LENGTH 32
#define REQUEST_CODE_LENGTH 2
#define COMMAND_CODE_LENGTH 2
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// flash
FlashDevice* flash;
#define FLASH_RECORD_CAPACITY 400
#define FLASH_OVERFLOW_ADDRESS 819176

// assay
#define TEST_NUMBER_OF_RECORDS_ADDR 512
#define TEST_RECORD_LENGTH 2048
#define TEST_RECORD_START_ADDR 516
#define TEST_RECORD_PARAM_OFFSET 38
#define TEST_RECORD_BUFFER_OFFSET 106
#define TEST_RECORD_BUFFER_SIZE 1942

#define SENSOR_MAX_NUMBER_OF_SAMPLES 20
#define SENSOR_SAMPLE_LENGTH 15

// eeprom addresses
#define EEPROM_ADDR_DEFAULT_FLAG 0
#define EEPROM_ADDR_SERIAL_NUMBER 1
#define EEPROM_ADDR_CALIBRATION_STEPS 20

// params
#define PARAM_CODE_LENGTH 3
#define MAX_NUMBER_OF_PARAMS 16
#define NUMBER_OF_PARAMS 8
#define PARAM_TOTAL_LENGTH (NUMBER_OF_PARAMS * 4)

// status
#define STATUS_LENGTH 623
#define STATUS(...) snprintf(spark_status, STATUS_LENGTH, __VA_ARGS__)

// device
#define SPARK_REGISTER_SIZE 623
#define SPARK_ARG_SIZE 63
#define SPARK_RESET_STAGE_STEPS -60000

// BCODE
#define MAX_BCODE_BUFFER_SIZE 2000
#define BCODE_PAYLOAD_INDEX 37
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_TEST_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)

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
char test_uuid[UUID_LENGTH + 1];
int test_start_time;
bool collect_sensor_data;
bool cancel_process;

// BCODE globals
char BCODE_buffer[MAX_BCODE_BUFFER_SIZE];
int BCODE_length;
int BCODE_count;
int BCODE_packets;
int BCODE_index;
char BCODE_test_uuid[UUID_LENGTH + 1];

// sensors
char test_result[2 * ASSAY_MAX_NUMBER_OF_SAMPLES][ASSAY_SAMPLE_LENGTH];
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
    bool pending;
    char arg[SPARK_ARG_SIZE + 1];
    char uuid[UUID_LENGTH + 1];
    int code;
    char param[SPARK_ARG_SIZE - UUID_LENGTH - REQUEST_CODE_LENGTH + 1];
    Request() {
        pending = false;
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

    // reserved
    int reserved[MAX_NUMBER_OF_PARAMS - NUMBER_OF_PARAMS];

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
    }
} brevitest;

struct BrevitestTestRecord{
    uint16_t num;
    uint16_t start_time;
    char uuid[32];
    Param param;
    uint8_t BCODE_version;
    uint8_t num_samples;
    uint16_t BCODE_length;
    char buffer[TEST_RECORD_BUFFER_SIZE];
};

struct BrevitestSensorRecord {
    char sensor_code;
    uint8_t reading_number;
    int reading_time;
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};
