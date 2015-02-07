#include "TCS34725.h"
#include "sparkWebsocket.h"
#include "SoftI2CMaster.h"
#include "sha1.h"
#include "Base64.h"
#include "dnsclient.h"
#include "global.h"

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
TCS34725 tcsAssay = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinAssaySDA, pinAssaySCL);
TCS34725 tcsControl = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinControlSDA, pinControlSCL);

// messaging
String message_types[9] = { "ID","INIT","PONG","PING","ECHO","RUN","RESET","DEVICE_STATUS","LAST_READING" };
int number_of_message_types = 9;
String message_queue[10];
int message_queue_length = 0;
int percent_complete = 0;

// stepper
int stepDelay =  1800;  // in microseconds
long stepsPerRaster = 100;

// solenoid
int solenoidSustainPower = 100;
int solenoidSurgePower = 255;
int solenoidSurgePeriod = 200; // milliseconds

// websocket
String wakToken;
IPAddress dnsServerIP(8,8,8,8);  			//Set the DNS server to use for the lookup
DNSClient dns;  							//This sets up an instance of the DNSClient
WebSocketClient websocketClient;
char websocketPath[] = "/websocket";
char websocketInit[] = "INIT:DEVICE:";
unsigned long ping_interval = 10000UL;
unsigned long last_ping = 0;

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

TCPClient client;

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

void send_signal_to_insert_cartridge() {
    int i;

    // drive solenoid 3 times
    for (i = 0; i < 3; i += 1) {
        solenoid_in(false);
        delay(300);
        solenoid_out();
        delay(300);
    }

    status_update(0, snprintf(status, 1000, "Open device, insert cartridge, and close device now"));
    // wait 10 seconds for insertion of cartridge
    for (i = 10; i > 0; i -= 1) {
        status_update(0, snprintf(status, 1000, "Assay will begin in %d seconds...", i));
        delay(1000);
    }
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

void read_sensor(TCS34725 *sensor, char *sensor_name) {
    uint16_t t, clear, red, green, blue;

    t = millis() - start_time;
    sensor->getRawData(&red, &green, &blue, &clear);

    status_update(0, snprintf(status, 1000, "Sensor:\t%s\tt:\t%u\tC:\t%u\tR:\t%u\tG:\t%u\tB:\t%u", sensor_name, t, clear, red, green, blue));
}

void get_sensor_readings() {
    char assay[6] = "Assay";
    char control[8] = "Control";
    analogWrite(pinLED, 20);
    delay(2000);

    for (int i = 0; i < 10; i += 1) {
        read_sensor(&tcsAssay, assay);
        read_sensor(&tcsControl, control);
        delay(1000);
    }

    analogWrite(pinLED, 0);
}

//
//
//  WEBSOCKET
//
//

bool openWebsocket() {
    char charSN[20];
    // Connect to the websocket server
    if (client.connect(serverIP, serverPort)) {
        status_update(0, snprintf(status, 1000, "Connected to websocket server"));

        // Handshake with the server
        websocketClient.path = websocketPath;
        websocketClient.host = serverName;

        if (websocketClient.handshake(&client)) {
            status_update(0, snprintf(status, 1000, "Websocket established."));
            serial_number.toCharArray(charSN, 20);
            send_data_to_websocket(strcat(websocketInit, charSN));
            start_time = millis();
            return true;
        }
        else {
            status_update(0, snprintf(status, 1000, "Handshake failed."));
            client.stop();
        }
    }
    else {
        status_update(0, snprintf(status, 1000, "Connection failed."));
    }

    return false;
}

void send_data_to_websocket(char *data) {
    websocketClient.sendData(data);
}

void send_string_to_websocket(String data) {
    websocketClient.sendData(data);
}

int add2buf(uint8_t *buf, int bufLen, String str) {
    for (unsigned int i = 0; i < str.length(); i += 1) {
        buf[bufLen] = str.charAt(i);
        bufLen++;
    }

    buf[bufLen] = '\0';
    return bufLen;
}

void pingWebsocket() {
    unsigned long elapsed_time = millis() - start_time;
    if ((elapsed_time - last_ping) > ping_interval) {
        last_ping = elapsed_time;
        send_string_to_websocket("PING");
    }
}

int queue_message(String msg) {
    char buf[100];

    msg.toCharArray(buf, 100);
    status_update(0, snprintf(status, 1000, buf));
    return 1;
}

void websocket_loop() {
    String data;
    int retries = 5;

    if (client.connected()) {

        websocketClient.getData(data);
        while (data.length() > 0) {
            status_update(0, snprintf(status, 1000, "Websocket message received from server."));
            queue_message(data);
            data = "";
            websocketClient.getData(data);
      }

      pingWebsocket();
    }
    else {
        status_update(0, snprintf(status, 1000, "Client disconnected. Will try to re-establish connection"));
        while (retries-- > 0) {
            if (openWebsocket()) {
                retries = 0;
            }
        }
    }
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

    msg = message_queue[0];
    message_queue_length -= 1;
    for (i = 0; i < message_queue_length; i += 1) {
        message_queue[i] = message_queue[i + 1];
    }

    getCommand(msg, &cmd);
    getParameters(msg, &param);

    for (i = 0; i < number_of_message_types; i += 1) {
        if (cmd.equals(message_types[i])) {
            break;
        }
    }

    switch (i) {
        case 0: // ID
            serial_number.toCharArray(buf, 20);
            status_update(0, snprintf(status, 1000, "Serial number: %s", buf));
            break;
        case 1: // INIT
            if (param.startsWith("SERVER")) {
                status_update(0, snprintf(status, 1000, "Server initialization received"));
            }
            break;
        case 2: // PING
        case 3: // PONG
        case 4: // ECHO
            cmd.toCharArray(buf, 50);
            status_update(0, snprintf(status, 1000, "Received: %s", buf));
            break;
        case 5: // RUN
            run_brevitest();
            break;
        case 6: // RESET
            reset_device();
            break;
        case 7: // DEVICE_STATUS
            status_update(0, snprintf(status, 1000, "Status: %s", status));
            break;
        case 8: // LAST_READING
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

    reset_x_stage();

    send_signal_to_insert_cartridge();

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
    case 1: // send to websocket
        send_data_to_websocket(status);
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
    Spark.variable("queuelength", &message_queue_length, INT);
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

    if (useDNSserver) {
        status_update(0, snprintf(status, 1000, "Host to lookup: %s", serverName));

        int ret = 0;
        dns.begin(dnsServerIP);					//this send the DNS server to the library
        ret = dns.getHostByName(serverName, serverIP);	//ret is the error code, getHostByName needs 2 args, the server and somewhere to save the Address
        if (ret != 1) {
            status_update(0, snprintf(status, 1000, "DNS failure. Error code: %d", ret));
                //    Error Codes
                //SUCCESS          1
                //TIMED_OUT        -1
                //INVALID_SERVER   -2
                //TRUNCATED        -3
                //INVALID_RESPONSE -4
        }
    }

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

    if (message_queue_length > 0) {
        process_message();
    }

    while (client.available() && len < 100) {
        buf[len] = client.read();
        incoming = true;
        len += 1;
    }
    buf[len] = '\0';
    if (incoming) {
        status_update(0, snprintf(status, 1000, "Message received: %s", buf));
    }

    delay(500);
}
