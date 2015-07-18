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
#define FLASH_RECORD_CAPACITY 1024
#define FLASH_ASSAY_COUNT_ADDR 262144
#define FLASH_ASSAY_FIRST_RECORD_ADDR 262148
#define FLASH_ASSAY_CAPACITY 200
#define FLASH_CARTRIDGE_COUNT_ADDR 524288
#define FLASH_CARTRIDGE_FIRST_RECORD_ADDR 524292
#define FLASH_CARTRIDGE_CAPACITY 1024
#define FLASH_OVERFLOW_ADDRESS (FLASH_CARTRIDGE_START_ADDR + FLASH_CARTRIDGE_CAPACITY * UUID_LENGTH)

// test
#define TEST_NUMBER_OF_RECORDS_ADDR 512
#define TEST_RECORD_LENGTH sizeof(test_record)
#define TEST_RECORD_START_ADDR (TEST_NUMBER_OF_RECORDS_ADDR + sizeof(int))
#define TEST_RECORD_UUID_OFFSET offsetof(struct BrevitestTestRecord, uuid)
#define TEST_RECORD_READING_STRING_LENGTH (63 - UUID_LENGTH)
#define TEST_DURATION_LENGTH 4

#define SENSOR_NUMBER_OF_SAMPLES 10
#define SENSOR_DELAY_BETWEEN_SAMPLES 500

// eeprom addresses
#define EEPROM_ADDR_FIRMWARE_VERSION 0
#define EEPROM_ADDR_FIRMWARE_VERSION_LENGTH 2
#define EEPROM_ADDR_FLASH_DATA_FORMAT_VERSION 2
#define EEPROM_ADDR_FLASH_DATA_FORMAT_VERSION_LENGTH 2
#define EEPROM_ADDR_SERIAL_NUMBER 4
#define EEPROM_ADDR_SERIAL_NUMBER_LENGTH 20
#define EEPROM_ADDR_CALIBRATION_STEPS 24
#define EEPROM_ADDR_CALIBRATION_STEPS_LENGTH 2

// params
#define PARAM_CODE_LENGTH 3
#define PARAM_CAPACITY 16
#define PARAM_COUNT 10
#define PARAM_TOTAL_LENGTH sizeof(Param)

// status
#define STATUS_LENGTH 622
#define STATUS(...) snprintf(spark_status, STATUS_LENGTH, __VA_ARGS__)

// device
#define SPARK_REGISTER_SIZE 622
#define SPARK_ARG_SIZE 63
#define SPARK_RESET_STAGE_STEPS -14000

// BCODE
#define BCODE_CAPACITY 489
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)
#define BCODE_PAYLOAD_INDEX (BCODE_UUID_INDEX + UUID_LENGTH)

// assay
#define ASSAY_NAME_MAX_LENGTH 64
#define ASSAY_STANDARD_CURVE_MAX_POINTS 20

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
int test_start_time;
int test_num;
int test_duration;
int test_progress;
int test_percent_complete;
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
        solenoid_surge_period_ms = 300; // milliseconds
        solenoid_sustain_power = 200;

        // sensors
        sensor_params = (TCS34725_GAIN_4X << 8) + TCS34725_INTEGRATIONTIME_50MS;
        sensor_ms_between_samples = 1000;
        sensor_led_power = 20;
        sensor_led_warmup_ms = 10000;
    }
} brevitest;

struct BrevitestSensorSampleRecord {
    char sensor_code;
    uint8_t sample_number;
    int sample_time;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
};
BrevitestSensorSampleRecord assay_buffer[SENSOR_NUMBER_OF_SAMPLES];
BrevitestSensorSampleRecord control_buffer[SENSOR_NUMBER_OF_SAMPLES];

struct BrevitestSensorRecord {
    char sensor_code;
    uint8_t number;
    int start_time;
    uint16_t red_norm;
    uint16_t green_norm;
    uint16_t blue_norm;
};

struct BrevitestTestRecord{
    uint16_t num;
    int start_time;
    int finish_time;
    char uuid[UUID_LENGTH];
    uint8_t BCODE_version;
    uint16_t BCODE_length;
    uint8_t integration_time;
    uint8_t gain;
    Param param;
    BrevitestSensorRecord sensor_reading_initial_assay;
    BrevitestSensorRecord sensor_reading_initial_control;
    BrevitestSensorRecord sensor_reading_final_assay;
    BrevitestSensorRecord sensor_reading_final_control;
    char BCODE[BCODE_CAPACITY];
} test_record;

struct BrevitestStandardCurvePoint {
    double x;
    double y;
};

struct BrevitestStandardCurve {
    uint16_t number_of_points;
    BrevitestStandardCurvePoint point[ASSAY_STANDARD_CURVE_MAX_POINTS];
}

struct BrevitestAssayRecord {
    char id[UUID_LENGTH];
    char name[ASSAY_NAME_MAX_LENGTH];
    double redMax;
    double greenMax;
    double greenMin;
    double redMin;
    BrevitestStandardCurve standardCurve;
};
