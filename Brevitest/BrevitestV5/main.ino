#include "TCS34725.h"
#include "SoftI2CMaster.h"
#include "dnsclient/dnsclient.h"
#include "global.h"
#include "Base64.h"
#include "sha1.h"
#include "sparkWebsocket.h"

// GLOBAL VARIABLES

// general
char serial_number[20];
unsigned long start_time;

// status
#define STATUS_LENGTH 1000
#define STATUS(args...) status_update(0, snprintf(status, STATUS_LENGTH, "STATUS ", args))
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
int stepDelay =  2000;  // in microseconds
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

#define CLOUD
#ifdef CLOUD
    char serverName[] = "brevitest.us.wak-apps.com";
    IPAddress serverIP;
    int serverPort = 80;
    bool useDNSserver = true;
#else
    char serverName[] = "brevitest.us.wak-apps.com";
    IPAddress serverIP(172, 16, 121, 53);
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
    STATUS( "Resetting X stage");
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

    STATUS("Open device, insert cartridge, and close device now");
    // wait 10 seconds for insertion of cartridge
    for (i = 10; i > 0; i -= 1) {
        STATUS("Assay will begin in %d seconds...", i);
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

void move_to_next_well_and_raster(int path_length, int well_size, String well_name) {
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

void read_sensor(TCS34725 *sensor, String sensor_name) {
    uint16_t clear, red, green, blue;

    sensor->getRawData(&red, &green, &blue, &clear);

    STATUS("Sensor:\t%s\tt:\t%d\tC:\t%f\tR:\t%f\tG:\t%f\tB:\t", sensor_name, millis(), clear, red, green, blue);
}

void get_sensor_readings() {
    analogWrite(pinLED, 20);
    delay(2000);

    for (int i = 0; i < 10; i += 1) {
        read_sensor(&tcsAssay, "Assay");
        read_sensor(&tcsControl, "Control");
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
    // Connect to the websocket server
    if (client.connect(serverIP, serverPort)) {
        STATUS("Connected to websocket server");

        // Handshake with the server
        websocketClient.path = websocketPath;
        websocketClient.host = serverName;

        if (websocketClient.handshake(&client)) {
            STATUS("Websocket established.");
            send_data_to_websocket((String) strcat(websocketInit, serial_number));
            start_time = millis();
            return true;
        }
        else {
            STATUS("Handshake failed.");
            client.stop();
        }
    }
    else {
        STATUS("Connection failed.");
    }

    return false;
}

void send_data_to_websocket(String data) {
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
        send_data_to_websocket("PING");
    }
}

void websocket_loop() {
    String data;
    int retries = 5;

    if (client.connected()) {

        websocketClient.getData(data);
        while (data.length() > 0) {
            STATUS("Websocket message received from server.");
            queue_message(data);
            data = "";
            websocketClient.getData(data);
      }

      pingWebsocket();
    }
    else {
        STATUS("Client disconnected. Will try to re-establish connection");
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
            STATUS("Serial number: %s", serial_number);
            break;
        case 1: // INIT
            if (param.startsWith("SERVER")) {
                STATUS("Server initialization received");
            }
            break;
        case 2: // PING
        case 3: // PONG
        case 4: // ECHO
            STATUS("Received: %s", cmd);
            break;
        case 5: // RUN
            run_brevitest();
            break;
        case 6: // RESET
            reset_device();
            break;
        case 7: // DEVICE_STATUS
            STATUS("Status: %s", status);
            break;
        case 8: // LAST_READING
            get_sensor_readings();
            break;
        default:
            STATUS("Invalid or unknown message.");
    }
}

//
//
//  COMMANDS
//
//

void run_brevitest() {
    STATUS("Running BreviTest...");

    reset_x_stage();

    send_signal_to_insert_cartridge();

    move_to_next_well_and_raster(2000, 10, "sample");
    move_to_next_well_and_raster(1000, 10, "antibody");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 10, "enzyme");
    move_to_next_well_and_raster(1000, 10, "buffer");
    move_to_next_well_and_raster(1000, 14, "indicator");

    STATUS("Reading sensors");
    get_sensor_readings();

    STATUS("Clean up");
    reset_device();

    STATUS("BreviTest run complete.");
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

void status_update(int mode, String status) {
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
  }
}

//
//
//  SETUP
//
//

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

    if (tcsAssay.begin()) {
        STATUS("Assay sensor initialized");
    }
    else {
        STATUS("Assay sensor not found");
    }

    if (tcsControl.begin()) {
        STATUS("Control sensor initialized");
    }
    else {
        STATUS("Control sensor not found");
    }

    start_time = millis();

    if (useDNSserver) {
        STATUS("Host to lookup: %s", serverName);

        int ret = 0;
        dns.begin(dnsServerIP);					//this send the DNS server to the library
        ret = dns.getHostByName(serverName, serverIP);	//ret is the error code, getHostByName needs 2 args, the server and somewhere to save the Address
        if (ret == 1) {
            STATUS("Hostname: %s, IP Address: %s", serverName, serverIP);
        }
        else {
            STATUS("DNS failure. Error code: %d", ret);
                //    Error Codes
                //SUCCESS          1
                //TIMED_OUT        -1
                //INVALID_SERVER   -2
                //TRUNCATED        -3
                //INVALID_RESPONSE -4
        }
    }
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

    websocket_loop();

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
        STATUS("Message received: ", buf);
    }

    delay(500);
}
