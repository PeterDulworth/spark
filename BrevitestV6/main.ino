#include "TCS34725.h"
#include "SoftI2CMaster.h"

// GLOBAL VARIABLES

// general
String serial_number = "OBAD-BWUG-NCPA-WIAF";
unsigned long start_time;
char buf[623];

// status
#define STATUS_LENGTH 1000
char status[STATUS_LENGTH];
bool device_ready = false;
bool init_device = false;
bool run_assay = false;

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

// sensor objects
#define SENSOR_SAMPLES 10

TCS34725 tcsAssay = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinAssaySDA, pinAssaySCL);
TCS34725 tcsControl = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinControlSDA, pinControlSCL);
String assay_result[2*SENSOR_SAMPLES];

// messaging
String spark_register = "";
String uuid = "";
int percent_complete = 0;

// stepper
int stepDelay =  1800;  // in microseconds
long stepsPerRaster = 100;

// solenoid
int solenoidSustainPower = 100;
int solenoidSurgePower = 255;
int solenoidSurgePeriod = 200; // milliseconds

//#define CLOUD
#ifdef CLOUD
    char serverName[] = "brevitest.us.wak-apps.com";
    IPAddress serverIP;
    int serverPort = 80;
    bool useDNSserver = true;
#endif
#ifndef CLOUD
    char serverName[] = "brevitest.us.wak-apps.com";
    IPAddress serverIP(172, 16, 121, 212);
    int serverPort = 8081;
    bool useDNSserver = false;
#endif

//
//
//  STATUS
//
//

void status_update(int count) {
    if (count >= 0) {
        Serial.println(status);
    }
}

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

        if (i%20 == 0) {
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
    status_update(snprintf(status, STATUS_LENGTH,  "Resetting X stage"));
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
    status_update(snprintf(status, STATUS_LENGTH, "Moving to %s well", well_name));
    move_steps(path_length);

    status_update(snprintf(status, STATUS_LENGTH, "Rastering %s well", well_name));
    raster_well(well_size);
}

//
//
//  SENSORS
//
//

void read_sensor(TCS34725 *sensor, char *sensor_name, int reading_number) {
    uint16_t t, clear, red, green, blue;

    t = millis() - start_time;
    sensor->getRawData(&red, &green, &blue, &clear);

    status_update(snprintf(status, STATUS_LENGTH, "Sensor:\t%s\tt:\t%u\tC:\t%u\tR:\t%u\tG:\t%u\tB:\t%u", sensor_name, t, clear, red, green, blue));
    assay_result[reading_number] = String(sensor_name[0]) + " \tN:" + String(reading_number) + " \tt:" + String(t) + "\tC:" + String(clear) + "\tR:" + String(red) + "\tG:"  + String(green) + "\tB:" + String(blue);
}

void collect_sensor_readings() {
    char assay[6] = "Assay";
    char control[8] = "Control";

    analogWrite(pinLED, 20);
    delay(2000);

    for (int i = 0; i < 10; i += 1) {
        read_sensor(&tcsAssay, assay, i);
        read_sensor(&tcsControl, control, i);
        delay(1000);
    }

    analogWrite(pinLED, 0);
}

//
//
//  DATA REQUESTS
//
//

void get_sensor_data(String request) {
    int index = request.toInt();
    spark_register = assay_result[index];
}

void get_serial_number() {
    spark_register = "";
    for (int i = 0; i < 19; i += 1) {
        spark_register.setCharAt(i, (char) EEPROM.read(i));
    }
}

//
//
//  EXPOSED FUNCTIONS
//
//

int ping(String msg) {
    return 1;
}

int write_serial_number(String msg) {
    if (msg.length() == 19) {
        status_update(snprintf(status, STATUS_LENGTH, "Writing serial number."));
        for (int i = 0; i < 19; i += 1) {
            EEPROM.write(i, (uint8_t) msg.charAt(i));
        }
        return 1;
    }
    else {
        status_update(snprintf(status, STATUS_LENGTH, "Serial number not 19 characters. Writing serial number failed."));
        return -1;
    }
}

int request_data(String msg) {
    int request_type;
    String request;

    msg.toCharArray(buf, 63);
    status_update(snprintf(status, STATUS_LENGTH, "Data request: %s", buf));
    if (spark_register.length() == 0) {
        uuid = "" + msg.substring(0, 31);
        Serial.println(uuid);
        request_type = msg.substring(32, 33).toInt();
        Serial.println(request_type);
        request = "" + msg.substring(34);
        Serial.println(request);

        switch (request_type) {
            case 0: // serial_number
                get_serial_number();
                break;
            case 1: // sensor data request
                get_sensor_data(request);
                break;
        }
        return (spark_register == "" ? -1 : 1);
    }
    else {
        spark_register.toCharArray(buf, 622);
        status_update(snprintf(status, STATUS_LENGTH, "Register locked. Request denied. Register: %s", buf));
        return -1;
    }
}

int release_data(String msg) {
    Serial.println(msg);
    if (uuid.equals(msg)) {
        spark_register = "";
        return 1;
    }
    else {
        status_update(snprintf(status, STATUS_LENGTH, "Register uuid mismatch. Release denied."));
        return -1;
    }
}

int initialize_device(String msg) {
    init_device = !run_assay;
    return 1;
}

int run_brevitest(String msg) {
    if (run_assay) {
        status_update(snprintf(status, STATUS_LENGTH, "Assay already running."));
        return -1;
    }
    if (!device_ready) {
        status_update(snprintf(status, STATUS_LENGTH, "Device not ready. Please reset the device."));
        return -1;
    }
    run_assay = true;
    return 1;
}

//
//
//  SETUP
//
//

void setup() {
    Spark.function("ping", ping);
    Spark.function("writeserial", write_serial_number);
    Spark.function("requestdata", request_data);
    Spark.function("releasedata", release_data);
    Spark.function("initdevice", initialize_device);
    Spark.function("runassay", run_brevitest);
    Spark.variable("register", &spark_register, STRING);
    Spark.variable("status", &status, STRING);

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

    if (tcsAssay.begin()) {
        status_update(snprintf(status, STATUS_LENGTH, "Assay sensor initialized"));
    }
    else {
        status_update(snprintf(status, STATUS_LENGTH, "Assay sensor not found"));
    }

    if (tcsControl.begin()) {
        status_update(snprintf(status, STATUS_LENGTH, "Control sensor initialized"));
    }
    else {
        status_update(snprintf(status, STATUS_LENGTH, "Control sensor not found"));
    }

    start_time = millis();
    status_update(snprintf(status, STATUS_LENGTH, "Hostname: %s, IP Address: %d.%d.%d.%d", serverName, serverIP[0], serverIP[1], serverIP[2], serverIP[3]));
}

//
//
//  LOOP
//
//

void loop(){
    if (init_device) {
        status_update(snprintf(status, STATUS_LENGTH, "Initializing device"));
        solenoid_out();
        reset_x_stage();
        device_ready = true;
        init_device = false;
    }

    if (device_ready && run_assay) {
        device_ready = false;
        status_update(snprintf(status, STATUS_LENGTH, "Running BreviTest..."));

        move_to_next_well_and_raster(2000, 10, "sample");
        move_to_next_well_and_raster(1000, 10, "antibody");
        move_to_next_well_and_raster(1000, 10, "buffer");
        move_to_next_well_and_raster(1000, 10, "enzyme");
        move_to_next_well_and_raster(1000, 10, "buffer");
        move_to_next_well_and_raster(1000, 14, "indicator");

        status_update(snprintf(status, STATUS_LENGTH, "Reading sensors"));
        collect_sensor_readings();

        status_update(snprintf(status, STATUS_LENGTH, "Clean up"));
        reset_x_stage();
        status_update(snprintf(status, STATUS_LENGTH, "BreviTest run complete."));
        run_assay = false;
    }

    delay(500);
}
