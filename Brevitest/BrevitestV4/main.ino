#include "TCS34725.h"
#include "SoftI2CMaster.h"

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

TCS34725 tcsAssay = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinAssaySDA, pinAssaySCL);
TCS34725 tcsControl = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinControlSDA, pinControlSCL);

int stepDelay =  2000;  // in microseconds
unsigned long lastRun = 0;
unsigned long runInterval = 15000;

long stepsPerRaster = 100;

int solenoidSustainPower = 100;
int solenoidSurgePower = 255;
int solenoidSurgePeriod = 200;

void setup() {
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
//    while(!Serial.available()) {
//        Spark.process();
//    }

    if (tcsAssay.begin()) {
        Serial.println("Assay sensor initialized");
    }
    else {
        Serial.println("Assay sensor not found");
    }

    if (tcsControl.begin()) {
        Serial.println("Control sensor initialized");
    }
    else {
        Serial.println("Control sensor not found");
    }
}

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

bool limitSwitchOn() {
    return (digitalRead(A0) == LOW);
}

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
//    Serial.println("Putting stepper to sleep");
    digitalWrite(pinStepperSleep, LOW);
}

void wake_stepper() {
//    Serial.println("Waking stepper");
    digitalWrite(pinStepperSleep, HIGH);
    delay(5);
}

void reset_x_stage() {
    Serial.println("Resetting X stage");
    move_steps(-30000);
}

void send_signal_to_insert_cartridge() {
    int i;

    // drive solenoid 3 times
    for (i = 0; i < 3; i += 1) {
        solenoid_in(false);
        delay(300);
        solenoid_out();
        delay(300);
    }

    Serial.println("Open device, insert cartridge, and close device now");
    // wait 10 seconds for insertion of cartridge
    for (i = 10; i > 0; i -= 1) {
        Serial.print ("Assay will begin in ");
        Serial.print(i);
        Serial.println(" seconds...");
        delay(1000);
    }
}

void move_to_next_well_and_raster(int path_length, int well_size, String well_name) {
    Serial.print("Moving to ");
    Serial.print(well_name);
    Serial.println(" well");
    move_steps(path_length);

    Serial.print("Rastering ");
    Serial.print(well_name);
    Serial.println(" well");
    raster_well(well_size);
}

void run_brevitest() {
    Serial.println("Running BreviTest...");

    reset_x_stage();

    send_signal_to_insert_cartridge();

    move_to_next_well_and_raster(2000, 10, "sample");
    move_to_next_well_and_raster(1000, 10, "antibody");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 10, "enzyme");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 14, "indicator");

    Serial.println("Reading sensors");
    bt_read_sensors();

    Serial.println("Clean up");
    bt_cleanup();

    Serial.println("BreviTest run complete.");
}

void bt_move_to_second_well() {
//  Serial.println("Moving to second well");
    move_steps(3000);
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

void read_sensor(TCS34725 *sensor, String sensor_name) {
    uint16_t clear, red, green, blue;

    sensor->getRawData(&red, &green, &blue, &clear);

    Serial.print("Sensor:\t"); Serial.print(sensor_name);
    Serial.print("\tt:\t"); Serial.print(millis());
    Serial.print("\tC:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.println(blue);
}

void bt_read_sensors() {
    analogWrite(pinLED, 20);
    delay(2000);

    for (int i = 0; i < 10; i += 1) {
        read_sensor(&tcsAssay, "Assay");
        read_sensor(&tcsControl, "Control");
        delay(1000);
    }

    analogWrite(pinLED, 0);
}

void bt_cleanup() {
    solenoid_out();
    reset_x_stage();
}

void loop(){
    run_brevitest();

    while (true) {
        Spark.process();
    }
}
