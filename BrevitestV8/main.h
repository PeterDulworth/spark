#include "TCS34725.h"
#include "SoftI2CMaster.h"
#include "flashee-eeprom.h"
    using namespace Flashee;

// GLOBAL VARIABLES AND DEFINES

#define FIRMWARE_VERSION 0x08

// flash
FlashDevice* flash;
#define FLASH_RECORD_CAPACITY 512
#define FLASH_OVERFLOW_ADDRESS 545284

// assay
#define ASSAY_NUMBER_OF_RECORDS_ADDR 512
#define ASSAY_RECORD_HEADER_LENGTH 384
#define ASSAY_RECORD_HEADER_START_ADDR 516
#define ASSAY_RECORD_HEADER_PARAM_OFFSET 48
#define ASSAY_RECORD_DATA_LENGTH 640
#define ASSAY_RECORD_START_ADDR 197124

#define ASSAY_NUMBER_OF_SAMPLES 10
#define ASSAY_SAMPLE_LENGTH 31

// eeprom addresses
#define EEPROM_ADDR_DEFAULT_FLAG 0
#define EEPROM_ADDR_SERIAL_NUMBER 1
#define EEPROM_ADDR_NUMBER_OF_STORED_ASSAYS 20

// general constants
#define SERIAL_NUMBER_LENGTH 19
#define UUID_LENGTH 32
#define REQUEST_CODE_LENGTH 2
#define COMMAND_CODE_LENGTH 2
#define PARAM_CODE_LENGTH 3
#define MAX_NUMBER_OF_PARAMS 64
#define NUMBER_OF_PARAMS 36
#define PARAM_TOTAL_LENGTH (NUMBER_OF_PARAMS * 4)

// status
#define STATUS_LENGTH 623
#define STATUS(...) snprintf(spark_status, STATUS_LENGTH, __VA_ARGS__)
#define ERROR_MESSAGE(err) Serial.println(err)
#define CANCELLABLE(x) if (!cancel_process) {x}

// device
#define SPARK_REGISTER_SIZE 623
#define SPARK_ARG_SIZE 63
#define MAX_BCODE_BUFFER_SIZE 2000
#define BCODE_PAYLOAD_INDEX 37
#define BCODE_NUM_LENGTH 3
#define BCODE_LEN_LENGTH 2
#define BCODE_TEST_UUID_INDEX (BCODE_NUM_LENGTH + BCODE_LEN_LENGTH)
#define BCODE_COMMAND_LENGTH 2

// pin definitions
int pinSolenoid = A1;
int pinStepperStep = D2;
int pinStepperDir = D1;
int pinStepperSleep = D0;
int pinLimitSwitch = A0;
int pinLED = A4;
int pinAssaySDA = D3;
int pinAssaySCL = D4;
int pinControlSDA = D5;
int pinControlSCL = D6;

// global variables
bool device_ready;
bool init_device;
bool run_assay;
char assay_uuid[UUID_LENGTH + 1];
int assay_start_time;
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
char assay_result[2 * ASSAY_NUMBER_OF_SAMPLES][ASSAY_SAMPLE_LENGTH];
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
    int step_delay_raster_us;  // microseconds
    int step_delay_transit_us;  // microseconds
    int step_delay_reset_us;  // microseconds
    int steps_per_raster;
    int stepper_wifi_ping_rate;
    int stepper_wake_delay_ms; // milliseconds

    // solenoid
    int solenoid_surge_power;
    int solenoid_surge_period_ms; // milliseconds
    int solenoid_sustain_power;
    int solenoid_off_ms;
    int solenoid_on_ms;
    int solenoid_first_off_ms;
    int solenoid_first_on_ms;
    int solenoid_finish_well_ms;
    int solenoid_cycles_per_raster;

    // sensors
    int sensor_params;
    int sensor_number_of_collections;
    int sensor_ms_between_samples;
    int led_power;
    int led_warmup_ms;

    // device
    int steps_to_reset;
    int steps_to_sample_well;
    int sample_well_rasters;
    int steps_to_antibody_well;
    int antibody_well_rasters;
    int steps_to_first_buffer_well;
    int first_buffer_well_rasters;
    int steps_to_enzyme_well;
    int enzyme_well_rasters;
    int steps_to_second_buffer_well;
    int second_buffer_well_rasters;
    int steps_to_indicator_well;
    int indicator_well_rasters;

    int step_delay_meniscus_us;
    int steps_for_meniscus_transition;
    int solenoid_start_well_ms;

    // reserved
    int reserved[MAX_NUMBER_OF_PARAMS - NUMBER_OF_PARAMS];

    Param() {
        //  DEFAULT VALUES
        // stepper
        step_delay_raster_us = 1800;  // microseconds
        step_delay_transit_us = 2000;  // microseconds
        step_delay_reset_us = 1200;  // microseconds, fastest mode
        steps_per_raster = 100;
        stepper_wifi_ping_rate = 100;
        stepper_wake_delay_ms = 5; // milliseconds

        // solenoid
        solenoid_surge_power = 255;
        solenoid_surge_period_ms = 200; // milliseconds
        solenoid_sustain_power = 100;
        solenoid_off_ms = 250;
        solenoid_on_ms = 700;
        solenoid_first_off_ms = 500;
        solenoid_first_on_ms = 2200;
        solenoid_finish_well_ms = 4000;
        solenoid_cycles_per_raster = 5;

        // sensors
        sensor_params = (TCS34725_INTEGRATIONTIME_50MS << 8) + TCS34725_GAIN_4X;
        sensor_number_of_collections = 1;
        sensor_ms_between_samples = 1000;
        led_power = 20;
        led_warmup_ms = 10000;

        // device
        steps_to_reset = -30000;
        steps_for_meniscus_transition = 200;
        steps_to_sample_well = 4200;
        sample_well_rasters = 10;
        steps_to_antibody_well = 1000;
        antibody_well_rasters = 10;
        steps_to_first_buffer_well = 1000;
        first_buffer_well_rasters = 10;
        steps_to_enzyme_well = 1000;
        enzyme_well_rasters = 10;
        steps_to_second_buffer_well = 1000;
        second_buffer_well_rasters = 10;
        steps_to_indicator_well = 1000;
        indicator_well_rasters = 14;

        step_delay_meniscus_us = 6000;  // microseconds
        steps_for_meniscus_transition = 200;
        solenoid_start_well_ms = 4000;
    }
} brevitest;

struct BrevitestHeader {
    int num;
    int data_addr;
    int data_length;
    int assay_start_time;
    char uuid[32];
    Param param;
    char reserved[80];
    BrevitestHeader() {
        num = 0;
        data_addr = 0;
        data_length = 0;
        assay_start_time = 0;
    }
};

struct BrevitestRecord {
    char sensor_code;
    char reading_number[2];
    char reading_time[11];
    char clear[5];
    char red[5];
    char green[5];
    char blue[5];
};
