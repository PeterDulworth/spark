// GLOBAL VARIABLES AND DEFINES

// flash
FlashDevice* flash;
#define MAX_RESULTS_STORED_IN_FLASH 1024
#define ASSAY_RECORD_HEADER_SIZE 64
#define ASSAY_RECORD_HEADER_START_ADDR 512
#define ASSAY_RECORD_HEADER_ADDR_OFFSET 36
#define ASSAY_RECORD_HEADER_LENGTH_OFFSET 40
#define ASSAY_RECORD_HEADER_TIME_OFFSET 44
#define ASSAY_RECORD_SIZE 32
#define ASSAY_RECORD_START_ADDR 66048
#define NUMBER_OF_SENSOR_SAMPLES 10

// eeprom addresses
#define EEPROM_ADDR_DEFAULT_FLAG 0
#define EEPROM_ADDR_SERIAL_NUMBER 1
#define EEPROM_ADDR_NUMBER_OF_STORED_ASSAYS 20

// general constants
#define SERIAL_NUMBER_LENGTH 19
#define UUID_LENGTH 32
#define REQUEST_CODE_LENGTH 2
#define COMMAND_CODE_LENGTH 2

// status
#define STATUS_LENGTH 623
char status[STATUS_LENGTH];
#define STATUS(...) snprintf(status, STATUS_LENGTH, __VA_ARGS__)
#define ERROR(err) Serial.println(err)

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
String uuid;
bool device_ready;
bool init_device;
bool run_assay;

// sensors
#define SENSOR_RECORD_LENGTH 28
char assay_result[2 * NUMBER_OF_SENSOR_SAMPLES][SENSOR_RECORD_LENGTH + 1];

TCS34725 tcsAssay;
TCS34725 tcsControl;

// spark messaging
#define SPARK_REGISTER_SIZE 623
#define SPARK_ARG_SIZE 63
char spark_register[SPARK_REGISTER_SIZE];
char spark_argument[SPARK_ARG_SIZE + 1];

struct Param {
    // stepper
    int step_delay_us;  // microseconds
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
    tcs34725IntegrationTime_t integration_time;
    tcs34725Gain_t gain;
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

    Param()
    {
        //  DEFAULT VALUES
        // stepper
        step_delay_us = 1800;
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
        integration_time = TCS34725_INTEGRATIONTIME_50MS;
        gain = TCS34725_GAIN_4X;
        sensor_number_of_collections = 1;
        sensor_ms_between_samples = 1000;
        led_power = 20;
        led_warmup_ms = 10000;

        // device
        steps_to_reset = -30000;
        steps_to_sample_well = 6000;
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
    }
} brevitest;
