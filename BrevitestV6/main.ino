#include "TCS34725.h"
#include "SoftI2CMaster.h"
 #include "flashee-eeprom.h"
    using namespace Flashee;

#include "main.h"

//
//
//  SOLENOID
//
//

void solenoid_in(bool check_limit_switch) {
    Spark.process();
    if (!(check_limit_switch && limitSwitchOn())) {
        analogWrite(pinSolenoid, brevitest.solenoid_surge_power);
        delay(brevitest.solenoid_surge_period_ms);
        analogWrite(pinSolenoid, brevitest.solenoid_sustain_power);
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

void move_steps(long steps, int step_delay){
    //rotate a specific number of steps - negative for reverse movement

    wake_stepper();

    int dir = (steps > 0)? LOW:HIGH;
    steps = abs(steps);

    digitalWrite(pinStepperDir,dir);

    for(long i = 0; i < steps; i += 1) {
        if (dir == HIGH && limitSwitchOn()) {
            break;
        }

        if (i % brevitest.stepper_wifi_ping_rate == 0) {
            Spark.process();
        }

        digitalWrite(pinStepperStep, HIGH);
        delayMicroseconds(step_delay);

        digitalWrite(pinStepperStep, LOW);
        delayMicroseconds(step_delay);
    }

    sleep_stepper();
}

void sleep_stepper() {
    digitalWrite(pinStepperSleep, LOW);
}

void wake_stepper() {
    digitalWrite(pinStepperSleep, HIGH);
    delay(brevitest.stepper_wake_delay_ms);
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
    move_steps(brevitest.steps_to_reset, brevitest.step_delay_reset_us);
}

void raster_well(int number_of_rasters) {
    for (int i = 0; i < number_of_rasters; i += 1) {
        if (limitSwitchOn()) {
            return;
        }
        move_steps(brevitest.steps_per_raster, brevitest.step_delay_raster_us);
        if (i < 1) {
            delay(brevitest.solenoid_first_off_ms);
            solenoid_in(true);
            delay(brevitest.solenoid_first_on_ms);
            solenoid_out();
        }
        else {
            for (int k = 0; k < (brevitest.solenoid_cycles_per_raster - 1); k += 1) {
                delay(brevitest.solenoid_off_ms);
                solenoid_in(true);
                delay(brevitest.solenoid_on_ms);
                solenoid_out();
            }
        }
    }
    delay(4000);
}

void move_to_next_well_and_raster(int path_length, int well_size, const char *well_name) {
    STATUS("Moving to %s well", well_name);
    move_steps(path_length, brevitest.step_delay_transit_us);

    STATUS("Rastering %s well", well_name);
    raster_well(well_size);
}

//
//
//  SENSORS
//
//

void init_sensor(TCS34725 *sensor, int sdaPin, int sclPin) {
    *sensor = TCS34725(brevitest.integration_time, brevitest.gain, sdaPin, sclPin);

    if (sensor->begin()) {
        sensor->enable();
    }
    else {
        ERROR("Sensor not found");
    }
}

void read_sensor(TCS34725 *sensor, char sensorCode, int reading_number) {
    uint16_t clear, red, green, blue;
    int t = Time.now();
    int index = 2 * reading_number + (sensorCode == 'A' ? 0 : 1);

    STATUS("Reading %s sensor (%d of %d)", (sensorCode == 'A' ? "Assay" : "Control"), reading_number + 1, NUMBER_OF_SENSOR_SAMPLES);

    sensor->getRawData(&red, &green, &blue, &clear);

    snprintf(&assay_result[index][0], SENSOR_RECORD_LENGTH, "%c%02d%05d%05u%05u%05u%05u", sensorCode, reading_number, t, clear, red, green, blue);
}

int get_flash_header_address(int count) {
    return ASSAY_RECORD_HEADER_START_ADDR + count * ASSAY_RECORD_HEADER_SIZE;
}

int get_flash_data_address(int count) {
    int addr = 0;
    int len = 0;
    int header;

    if (count > 0) {
        header = get_flash_header_address(count - 1);
        flash->read(&addr, header + ASSAY_RECORD_HEADER_ADDR_OFFSET, 4);
        flash->read(&len, header + ASSAY_RECORD_HEADER_LENGTH_OFFSET, 4);
    }

    return addr + len;
}

int get_flash_record_count() {
    int count = EEPROM.read(EEPROM_ADDR_NUMBER_OF_STORED_ASSAYS);
    if (count >= MAX_RESULTS_STORED_IN_FLASH) {
        ERROR("Spark out of flash storage space");
        return -1;
    }
    return count;
}

void write_sensor_readings_to_flash(int ref_time) {
    int count = get_flash_record_count();
    int header = get_flash_header_address(count);
    int addr = get_flash_data_address(count);
    int len = SENSOR_RECORD_LENGTH * NUMBER_OF_SENSOR_SAMPLES;

    count++;

    flash->write(&count, header, 4);
    flash->write(&addr, header + ASSAY_RECORD_HEADER_ADDR_OFFSET, 4);
    flash->write(&len, header + ASSAY_RECORD_HEADER_LENGTH_OFFSET, 4);
    flash->write(&ref_time, header + ASSAY_RECORD_HEADER_TIME_OFFSET, 4);

    for (int i = 0; i < NUMBER_OF_SENSOR_SAMPLES; i += 1) {
        flash->write(&assay_result[i][0], addr + i * ASSAY_RECORD_SIZE, ASSAY_RECORD_SIZE);
    }

    EEPROM.write(EEPROM_ADDR_NUMBER_OF_STORED_ASSAYS, count);
}

void collect_sensor_readings(int assay_time) {
    for (int i = 0; i < NUMBER_OF_SENSOR_SAMPLES; i += 1) {
        read_sensor(&tcsAssay, 'A', i);
        read_sensor(&tcsControl, 'C', i);
        delay(brevitest.sensor_ms_between_samples);
    }

    write_sensor_readings_to_flash(assay_time);
}

void recollect_sensor_data() {
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
    analogWrite(pinLED, brevitest.led_power);
    delay(brevitest.led_warmup_ms);

    collect_sensor_readings(Time.now());

    analogWrite(pinLED, 0);
    tcsAssay.disable();
    tcsControl.disable();
}

//
//
//  MAIN FUNCTIONS
//
//

void get_all_sensor_data() {
    int bufSize, i, len;
    int index = 0;

    for (i = 0; i < 2 * NUMBER_OF_SENSOR_SAMPLES; i += 1) {
        bufSize = SPARK_REGISTER_SIZE - index;
        if (bufSize < SENSOR_RECORD_LENGTH + 1) {
            break;
        }
        memcpy(&spark_register[index], &assay_result[i][0], SENSOR_RECORD_LENGTH);
        index += SENSOR_RECORD_LENGTH;
        spark_register[index++] = '\n';
    }
    spark_register[index] = '\0';
}

void get_archived_sensor_data(String request) {
    int bufSize, i;
    int req = request.toInt();
    int index = 0;
    int header = get_flash_header_address(req);
    int addr = get_flash_data_address(req);

    flash->read(&spark_register[index], header, ASSAY_RECORD_HEADER_SIZE);
    for (i = 0; i < 2 * NUMBER_OF_SENSOR_SAMPLES; i += 1) {
        bufSize = SPARK_REGISTER_SIZE - index;
        if (bufSize < ASSAY_RECORD_SIZE + 1) {
            break;
        }
        flash->read(&spark_register[index], addr, ASSAY_RECORD_SIZE);
        addr += ASSAY_RECORD_SIZE;
        index += ASSAY_RECORD_SIZE;
        spark_register[index++] = '\n';
    }
    spark_register[index] = '\0';
}

void get_sensor_data(String request) {
    int index = request.toInt();
    memcpy(spark_register, assay_result[index], SENSOR_RECORD_LENGTH);
    spark_register[SENSOR_RECORD_LENGTH] = '\n';
}

void get_serial_number() {
    spark_register[SERIAL_NUMBER_LENGTH] = '\0';
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        spark_register[EEPROM_ADDR_SERIAL_NUMBER + i] = (char) EEPROM.read(i);
    }
}

void get_params() {
    int len = sizeof(brevitest);
    flash->read(spark_register, 0, len);
    spark_register[len] = '\0';
}

int write_serial_number(String msg) {
    STATUS("Writing serial number");
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        EEPROM.write(EEPROM_ADDR_SERIAL_NUMBER + i, (uint8_t) msg.charAt(i));
    }
    return 1;
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
        uuid = "" + msg.substring(0, UUID_LENGTH);
        request_type = msg.substring(UUID_LENGTH, UUID_LENGTH + REQUEST_CODE_LENGTH).toInt();
        request = "" + msg.substring(UUID_LENGTH + REQUEST_CODE_LENGTH);

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
            case 3: // archived sensor data
                get_archived_sensor_data(request);
                break;
            case 4: // params
                get_params();
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
    command_type = msg.substring(0, COMMAND_CODE_LENGTH).toInt();
    arg = "" + msg.substring(COMMAND_CODE_LENGTH);

    switch (command_type) {
        case 0: // write serial number
            return write_serial_number(arg);
        case 1: // initialize device
            return initialize_device();
        case 2: // run assay
            return run_brevitest();
        case 3: // collect sensor data
            return recollect_sensor_data();
    }
    return -1;
}

int change_param(String msg) {
    int param_index, valueInt;
    String value;
    void *ptr;

    msg.toCharArray(spark_argument, SPARK_ARG_SIZE);
    STATUS("Changing parameter: %s", spark_argument);
    param_index = msg.substring(0, PARAM_CODE_LENGTH).toInt();
    if (param_index > MAX_PARAM_OFFSET) {
        ERROR("Parameter index out of range");
        return -1;
    }
    value = "" + msg.substring(PARAM_CODE_LENGTH);
    ptr = &brevitest + param_index;
    *ptr = value.toInt();

    flash->write(ptr, param_index, 4);

    return 1;
}


//
//
//  PARAMS
//
//

void write_default_params() {
    EEPROM.write(0, 0xA1);
    flash->write(&brevitest, 0, sizeof(brevitest));
}

void read_params() {
    flash->read(&brevitest, 0, sizeof(brevitest));
}

void load_params() {
    char c = EEPROM.read(0);
    if (c != (char) 0xA1) {
        Serial.println("Writing default params");
        write_default_params();
    }
    else {
        Serial.println("Reading params");
        read_params();
    }
}

//
//
//  SETUP
//
//

void setup() {
    Spark.function("runcommand", run_command);
    Spark.function("requestdata", request_data);
    Spark.function("changeparam", change_param);
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
    while (!Serial.available()) {
        Spark.process();
    }

    flash = Devices::createWearLevelErase();

    load_params();

    uuid = "";

    device_ready = false;
    init_device = false;
    run_assay = false;

    STATUS("Setup complete");
}

//
//
//  LOOP
//
//

void loop(){
    int assay_start_time;

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

        assay_start_time = Time.now();

        analogWrite(pinLED, brevitest.led_power);

        move_to_next_well_and_raster(brevitest.steps_to_sample_well, brevitest.sample_well_rasters, "sample");
        move_to_next_well_and_raster(brevitest.steps_to_antibody_well, brevitest.antibody_well_rasters, "antibody");
        move_to_next_well_and_raster(brevitest.steps_to_first_buffer_well, brevitest.first_buffer_well_rasters, "first buffer");
        move_to_next_well_and_raster(brevitest.steps_to_enzyme_well, brevitest.enzyme_well_rasters, "enzyme");
        move_to_next_well_and_raster(brevitest.steps_to_second_buffer_well, brevitest.second_buffer_well_rasters, "second buffer");
        move_to_next_well_and_raster(brevitest.steps_to_indicator_well, brevitest.indicator_well_rasters, "indicator");

        collect_sensor_readings(assay_start_time);

        STATUS("Finishing assay");
        analogWrite(pinLED, 0);
        tcsAssay.disable();
        tcsControl.disable();

        reset_x_stage();

        STATUS("Assay complete.");
        run_assay = false;
    }
}
