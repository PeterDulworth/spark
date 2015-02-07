#include "TCS34725.h"
#include "SoftI2CMaster.h"

// GLOBAL VARIABLES

// general
String serial_number = "OBAD-BWUG-NCPA-WIAF";
unsigned long start_time;

// status
#define STATUS_LENGTH 1000
char status[STATUS_LENGTH];

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
#define SENSOR_SAMPLES 10 // number of sensor readings per run

TCS34725 tcsAssay = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinAssaySDA, pinAssaySCL);
TCS34725 tcsControl = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinControlSDA, pinControlSCL);
String assay_result[2*SENSOR_SAMPLES];

// messaging
#define MESSAGE_TYPES 8
String message_types[MESSAGE_TYPES] = { "ID","PING","INIT_DEVICE","INIT_ASSAY","RUN_ASSAY","RESET","DEVICE_STATUS","LAST_READING" };
String incoming_queue[10];
int incoming_queue_length = 0;
String outgoing_queue[10];
int outgoing_queue_length = 0;
int percent_complete = 0;
bool device_ready = false;

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
    status_update(0, snprintf(status, 1000,  "Resetting X stage"));
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
    status_update(0, snprintf(status, 1000, "Moving to %s well", well_name));
    move_steps(path_length);

    status_update(0, snprintf(status, 1000, "Rastering %s well", well_name));
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

    status_update(0, snprintf(status, 1000, "Sensor:\t%s\tt:\t%u\tC:\t%u\tR:\t%u\tG:\t%u\tB:\t%u", sensor_name, t, clear, red, green, blue));
    assay_result[reading_number] = String(sensor_name[0]) + " \tt:" + String(t) + "\tC:" + String(clear) + "\tR:" + String(red) + "\tG:"  + String(green) + "\tB:" + String(blue);
}

void get_sensor_readings() {
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
//  MESSAGE QUEUEING
//
//

int queue_message(String msg) {
    char buf[100];

    msg.toCharArray(buf, 100);
    status_update(0, snprintf(status, 1000, buf));
    incoming_queue[incoming_queue_length++] = msg;
    return 1;
}

//
//
//  MESSAGE HANDLING
//
//

void getCommand(String msg, String *cmd) {
    *cmd = "";
    *cmd += msg.substring(0, msg.indexOf(":"));
    cmd->trim();
    cmd->toUpperCase();
}

void getParameters(String msg, String *param) {
    *param = "";
    *param += msg.substring(msg.indexOf(":") + 1);
    param->trim();
    param->toUpperCase();
}

void process_message() {
    String cmd, msg, param;
    char buf[50];
    int i;

    msg = incoming_queue[0];
    incoming_queue_length -= 1;
    for (i = 0; i < incoming_queue_length; i += 1) {
        incoming_queue[i] = incoming_queue[i + 1];
    }

    getCommand(msg, &cmd);
    getParameters(msg, &param);

    for (i = 0; i < MESSAGE_TYPES; i += 1) {
        if (cmd.equals(message_types[i])) {
            break;
        }
    }

    switch (i) {
        case 0: // ID
            serial_number.toCharArray(buf, 20);
            status_update(0, snprintf(status, 1000, "Serial number: %s", buf));
            break;
        case 1: // PING
            cmd.toCharArray(buf, 50);
            status_update(0, snprintf(status, 1000, "Received: %s", buf));
            break;
        case 2: // LOAD_DATA
            break;
        case 3: // CONFIRM_DATA
            break;
        case 4: // RUN_ASSAY
            if (device_ready) {
                device_ready = false;
                run_brevitest();
            }
            else {
                status_update(0, snprintf(status, 1000, "Device not ready. Please reset the device."));
            }
            break;
        case 5: // RESET
            status_update(0, snprintf(status, 1000, "Resetting device"));
            reset_device();
            device_ready = true;
            break;
        case 6: // DEVICE_STATUS
            status_update(0, snprintf(status, 1000, "Status: %s", status));
            break;
        case 7: // LAST_READING
            get_sensor_readings();
            break;
        default:
            status_update(0, snprintf(status, 1000, "Invalid or unknown message."));
    }
}

//
//
//  COMMANDS
//
//

void run_brevitest() {
    status_update(0, snprintf(status, 1000, "Running BreviTest..."));

    move_to_next_well_and_raster(2000, 10, "sample");
    move_to_next_well_and_raster(1000, 10, "antibody");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 10, "enzyme");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 14, "indicator");

    status_update(0, snprintf(status, 1000, "Reading sensors"));
    get_sensor_readings();

    status_update(0, snprintf(status, 1000, "Clean up"));
    reset_device();

    status_update(0, snprintf(status, 1000, "BreviTest run complete."));
}

void reset_device() {
    solenoid_out();
    reset_x_stage();
}

//
//
//  STATUS
//
//

void status_update(int mode, int count) {
    if (count < 0) {
        return;
    }

    switch (mode) {
    case 0: // send to serial port
        Serial.println(status);
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;
    }
}

//
//
//  SETUP
//
//

void setup() {
    Spark.function("queue", queue_message);
    Spark.variable("serialnumber", &serial_number, STRING);
    Spark.variable("status", &status, STRING);
    Spark.variable("queuelength", &incoming_queue_length, INT);
    Spark.variable("percentdone", &percent_complete, INT);

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
    while (!Serial.available()) {
        Spark.process();
    }

    if (tcsAssay.begin()) {
        status_update(0, snprintf(status, 1000, "Assay sensor initialized"));
    }
    else {
        status_update(0, snprintf(status, 1000, "Assay sensor not found"));
    }

    if (tcsControl.begin()) {
        status_update(0, snprintf(status, 1000, "Control sensor initialized"));
    }
    else {
        status_update(0, snprintf(status, 1000, "Control sensor not found"));
    }

    start_time = millis();

    status_update(0, snprintf(status, 1000, "Hostname: %s, IP Address: %d.%d.%d.%d", serverName, serverIP[0], serverIP[1], serverIP[2], serverIP[3]));
}

//
//
//  LOOP
//
//

void loop(){
    bool incoming = false;
    int len = 0;
    char buf[100];

//    websocket_loop();

    if (incoming_queue_length > 0) {
        process_message();
    }

    delay(500);
}
