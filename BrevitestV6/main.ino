#include "TCS34725.h"
#include "SoftI2CMaster.h"

// GLOBAL VARIABLES

// general
//String serial_number = "OBAD-BWUG-NCPA-WIAF";
unsigned long start_time;

// status
#define STATUS_LENGTH 623
char status[STATUS_LENGTH];
bool device_ready = false;
bool init_device = false;
bool run_assay = false;
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

// spark messaging
#define SPARK_REGISTER_SIZE 623
#define SPARK_ARG_SIZE 63
char spark_register[SPARK_REGISTER_SIZE];
char spark_argument[SPARK_ARG_SIZE + 1];
String uuid = "";

// stepper
int stepDelay =  1800;  // in microseconds
long stepsPerRaster = 100;

// solenoid
int solenoidSustainPower = 100;
int solenoidSurgePower = 255;
int solenoidSurgePeriod = 200; // milliseconds

// sensors
#define SENSOR_SAMPLES 10
#define SENSOR_MS_BETWEEN_SAMPLES 1000
String assay_result[2*SENSOR_SAMPLES];

tcs34725IntegrationTime_t integration_time = TCS34725_INTEGRATIONTIME_50MS;
tcs34725Gain_t gain = TCS34725_GAIN_4X;

TCS34725 tcsAssay, tcsControl;

//
//
//  SOLENOID
//
//

void solenoid_in(bool check_limit_switch) {
    Spark.process();
    if (!(check_limit_switch && limitSwitchOn())) {
        analogWrite(pinSolenoid, solenoidSurgePower);
        delay(solenoidSurgePeriod);
        analogWrite(pinSolenoid, solenoidSustainPower);
    }
}

void solenoid_out() {
    Spark.process();
    analogWrite(pinSolenoid, 0);
}

//
//
//  STEPPER
//
//

void move_steps(long steps){
    //rotate a specific number of steps - negative for reverse movement

    wake_stepper();

    int dir = (steps > 0)? LOW:HIGH;
    steps = abs(steps);

    digitalWrite(pinStepperDir,dir);

    for(long i = 0; i < steps; i += 1) {
        if (dir == HIGH && limitSwitchOn()) {
            break;
        }

        if (i%100 == 0) {
            Spark.process();
        }

        digitalWrite(pinStepperStep, HIGH);
        delayMicroseconds(stepDelay);

        digitalWrite(pinStepperStep, LOW);
        delayMicroseconds(stepDelay);
    }

    sleep_stepper();
}

void sleep_stepper() {
    digitalWrite(pinStepperSleep, LOW);
}

void wake_stepper() {
    digitalWrite(pinStepperSleep, HIGH);
    delay(5);
}

//
//
//  CONTROL
//
//

bool limitSwitchOn() {
    return (digitalRead(A0) == LOW);
}

void reset_x_stage() {
    STATUS("Resetting device");
    move_steps(-30000);
}

void raster_well(int number_of_rasters) {
    for (int i = 0; i < number_of_rasters; i += 1) {
        if (limitSwitchOn()) {
            return;
        }
        move_steps(stepsPerRaster);
        if (i < 1) {
            delay(500);
            solenoid_in(true);
            delay(2200);
            solenoid_out();
        }
        else {
            for (int k = 0; k < 4; k += 1) {
                delay(250);
                solenoid_in(true);
                delay(700);
                solenoid_out();
            }
        }
    }
    delay(4000);
}

void move_to_next_well_and_raster(int path_length, int well_size, const char *well_name) {
    STATUS("Moving to %s well", well_name);
    move_steps(path_length);

    STATUS("Rastering %s well", well_name);
    raster_well(well_size);
}

//
//
//  SENSORS
//
//

void init_sensor(TCS34725 *sensor, int sdaPin, int sclPin) {
    *sensor = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaPin, sclPin);

    if (sensor->begin()) {
        sensor->enable();
    }
    else {
        ERROR("Sensor not found");
    }
}

void read_sensor(TCS34725 *sensor, char sensorCode, int reading_number, char *buf) {
    uint16_t clear, red, green, blue;
    unsigned long t = millis() - start_time;
    int index = 2 * reading_number + (sensorCode == 'A' ? 0 : 1);

    STATUS("Reading %s sensor (%d of 10)", (sensorCode == 'A' ? "Assay" : "Control"), reading_number + 1);

    sensor->getRawData(&red, &green, &blue, &clear);

    snprintf(buf, 64, "%c\t%d\t%lu\t%u\t%u\t%u\t%u", sensorCode, reading_number, t, clear, red, green, blue);
    String temp(buf);
    assay_result[index] = temp;
}

void collect_sensor_readings() {
    char buf[64];
    start_time = millis();
    for (int i = 0; i < 10; i += 1) {
        read_sensor(&tcsAssay, 'A', i, buf);
        read_sensor(&tcsControl, 'C', i, buf);
        delay(SENSOR_MS_BETWEEN_SAMPLES);
    }
}

//
//
//  MAIN FUNCTIONS
//
//

void get_all_sensor_data() {
    int bufSize, i, len;
    int index = 0;

    for (i = 0; i < 2*SENSOR_SAMPLES; i += 1) {
        bufSize = SPARK_REGISTER_SIZE - index;
        len = assay_result[i].length();
        if (bufSize < len + 1) {
            break;
        }
        assay_result[i].toCharArray(&spark_register[index], bufSize);
        index += len;
        spark_register[index++] = '\n';
    }
    spark_register[index] = '\0';
}

void get_sensor_data(String request) {
    int index = request.toInt();
    assay_result[index].toCharArray(spark_register, SPARK_REGISTER_SIZE);
}

void get_serial_number() {
    spark_register[19] = '\0';
    for (int i = 0; i < 19; i += 1) {
        spark_register[i] = (char) EEPROM.read(i);
    }
}

int write_serial_number(String msg) {
    if (msg.length() == 19) {
        STATUS("Writing serial number");
        for (int i = 0; i < 19; i += 1) {
            EEPROM.write(i, (uint8_t) msg.charAt(i));
        }
        return 1;
    }
    else {
        ERROR("Writing serial number failed (Serial number must be 19 characters)");
        return -1;
    }
}

int initialize_device() {
    init_device = !run_assay;
    return 1;
}

int run_brevitest() {
    if (run_assay) {
        ERROR("Command ignored. Assay already running.");
        return -1;
    }
    if (!device_ready) {
        ERROR("Device not ready. Please reset the device.");
        return -1;
    }
    run_assay = true;
    return 1;
}

//
//
//  EXPOSED FUNCTIONS
//
//

int request_data(String msg) {
    int request_type;
    String request;

    msg.toCharArray(spark_argument, SPARK_ARG_SIZE);
    if (spark_register[0] == '\0') {
        STATUS("Data request: %s", spark_argument);
        uuid = "" + msg.substring(0, 32);
        request_type = msg.substring(32, 34).toInt();
        request = "" + msg.substring(34);

        switch (request_type) {
            case 0: // serial_number
                get_serial_number();
                break;
            case 1: // all sensor data
                get_all_sensor_data();
                break;
            case 2: // one sensor data point
                get_sensor_data(request);
                break;
        }
        return (spark_register[0] == '\0' ? -1 : 1);
    }
    else {
        if (uuid.equals(msg)) {
            spark_register[0] = '\0';
            return 1;
        }
        else {
            ERROR("Register uuid mismatch. Release denied.");
            return -1;
        }
    }
}

int run_command(String msg) {
    int command_type;
    String arg;

    msg.toCharArray(spark_argument, SPARK_ARG_SIZE);
    STATUS("Run command: %s", spark_argument);
    command_type = msg.substring(0, 2).toInt();
    arg = "" + msg.substring(2);

    switch (command_type) {
        case 0: // write serial number
            return write_serial_number(arg);
        case 1: // initialize device
            return initialize_device();
        case 2: // run assay
            return run_brevitest();
    }
    return -1;
}

int sensor_data(String msg) {
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
    analogWrite(pinLED, 20);
    delay(10000);

    collect_sensor_readings();

    analogWrite(pinLED, 0);
    tcsAssay.disable();
    tcsControl.disable();
    return 1;
}


//
//
//  SETUP
//
//

void setup() {
    Spark.function("runcommand", run_command);
    Spark.function("requestdata", request_data);
    Spark.function("sensordata", sensor_data);
    Spark.variable("register", spark_register, STRING);
    Spark.variable("status", status, STRING);

    pinMode(pinSolenoid, OUTPUT);
    pinMode(pinStepperStep, OUTPUT);
    pinMode(pinStepperDir, OUTPUT);
    pinMode(pinStepperSleep, OUTPUT);
    pinMode(pinLimitSwitch, INPUT_PULLUP);
    pinMode(pinLED, OUTPUT);
    pinMode(pinAssaySDA, OUTPUT);
    pinMode(pinAssaySCL, OUTPUT);
    pinMode(pinControlSDA, OUTPUT);
    pinMode(pinControlSCL, OUTPUT);

    digitalWrite(pinSolenoid, LOW);
    digitalWrite(pinStepperSleep, LOW);

    Serial.begin(9600);

    STATUS("Setup complete");
}

//
//
//  LOOP
//
//

void loop(){
    if (init_device) {
        STATUS("Initializing device");

        init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
        init_sensor(&tcsControl, pinControlSDA, pinControlSCL);

        solenoid_out();
        reset_x_stage();
        analogWrite(pinLED, 0);
        device_ready = true;
        init_device = false;
        STATUS("Device initialized and ready to run assay");
    }

    if (device_ready && run_assay) {
        device_ready = false;
        STATUS("Running assay...");

        start_time = millis();

        analogWrite(pinLED, 20);

        move_to_next_well_and_raster(6000, 10, "sample");
        move_to_next_well_and_raster(1000, 10, "antibody");
        move_to_next_well_and_raster(1000, 10, "buffer");
        move_to_next_well_and_raster(1000, 10, "enzyme");
        move_to_next_well_and_raster(1000, 10, "buffer");
        move_to_next_well_and_raster(1000, 14, "indicator");

        collect_sensor_readings();

        STATUS("Finishing assay");
        analogWrite(pinLED, 0);
        tcsAssay.disable();
        tcsControl.disable();

        reset_x_stage();

        STATUS("Assay complete.");
        run_assay = false;
    }
}
