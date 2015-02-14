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
    uint8_t it, gain;

    it = brevitest.sensor_params >> 8;
    gain = brevitest.sensor_params & 0x00FF;
    *sensor = TCS34725((tcs34725IntegrationTime_t) it, (tcs34725Gain_t) gain, sdaPin, sclPin);

    if (sensor->begin()) {
        sensor->enable();
    }
    else {
        ERROR_MESSAGE("Sensor not found");
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
        ERROR_MESSAGE("Spark out of flash storage space");
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

//
//
//  REQUESTS
//
//

void get_serial_number() {
    spark_register[SERIAL_NUMBER_LENGTH] = '\0';
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        spark_register[i] = (char) EEPROM.read(EEPROM_ADDR_SERIAL_NUMBER + i);
    }
}

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

void get_sensor_data() {
    int index = extract_int_from_argument(spark_request.param, 0, strlen(spark_request.param));
    memcpy(spark_register, assay_result[index], SENSOR_RECORD_LENGTH);
    spark_register[SENSOR_RECORD_LENGTH] = '\n';
}

void get_archived_sensor_data() {
    int bufSize, i;
    int index = 0;
    int num = extract_int_from_argument(spark_request.param, 0, strlen(spark_request.param));
    int header = get_flash_header_address(num);
    int addr = get_flash_data_address(num);

    flash->read(&spark_register[index], header, ASSAY_RECORD_HEADER_SIZE);
    index += ASSAY_RECORD_HEADER_SIZE;
    spark_register[index] = '\n';
    index++;

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

void get_one_param() {
    int value;

    Serial.println("get_one_param");
    int num = extract_int_from_argument(spark_request.param, 0, strlen(spark_request.param));
    flash->read(&value, num, 4);
    sprintf(spark_register, "%d", value);
    Serial.println(String(spark_register));
}

void get_all_params() {
    int value, len;
    int index = 0;

    for (int i = 0; i < PARAM_TOTAL_LENGTH; i += 4) {
        flash->read(&value, i, 4);
        len = sprintf(&spark_register[index], "%d,", value);
        index += len;
    }
    spark_register[--index] = '\0';
    Serial.println(String(spark_register));
}

//
//
//  COMMANDS
//
//

int write_serial_number() {
    STATUS("Writing serial number");
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        EEPROM.write(EEPROM_ADDR_SERIAL_NUMBER + i, spark_command.param[i]);
    }
    return 1;
}

int initialize_device() {
    init_device = !run_assay;
    return 1;
}

int run_brevitest() {
    if (run_assay) {
        ERROR_MESSAGE("Command ignored. Assay already running.");
        return -1;
    }
    if (!device_ready) {
        ERROR_MESSAGE("Device not ready. Please reset the device.");
        return -1;
    }
    run_assay = true;
    return 1;
}

int recollect_sensor_data() {
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
    analogWrite(pinLED, brevitest.led_power);
    delay(brevitest.led_warmup_ms);

    collect_sensor_readings(Time.now());

    analogWrite(pinLED, 0);
    tcsAssay.disable();
    tcsControl.disable();

    return 1;
}

int change_param() {
    int param_index;
    int *ptr;

    STATUS("Changing parameter value");
    param_index = extract_int_from_argument(spark_command.param, 0, PARAM_CODE_LENGTH);
    if (param_index >= PARAM_TOTAL_LENGTH) {
        ERROR_MESSAGE("Parameter index out of range");
        return -1;
    }
    ptr = (int *) &brevitest +  param_index;
    *ptr = extract_int_from_argument(spark_command.param, PARAM_CODE_LENGTH, strlen(spark_command.param));

    flash->write(ptr, param_index, 4);

    return *ptr;
}

int reset_params() {
    STATUS("Resetting parameters to default values");

    EEPROM.write(0, 0xA1);
    write_default_params();
    read_params();

    return 1;
}

//
//
//  EXPOSED FUNCTIONS
//
//

int request_data(String msg) {
    char new_uuid[UUID_LENGTH];

    if (spark_request.pending) {
        Serial.println("Request pending");
        msg.toCharArray(new_uuid, UUID_LENGTH + 1);
        if (strcmp(new_uuid, spark_request.uuid) == 0) {
            spark_register[0] = '\0';
            spark_request.pending = false;
            return 1;
        }
        else {
            ERROR_MESSAGE("Register uuid mismatch. Release denied.");
            return -1;
        }
    }
    else {
        Serial.println("Process request");
        parse_spark_request(msg);
        switch (spark_request.code) {
            case 0: // serial_number
                get_serial_number();
                break;
            case 1: // all sensor data
                get_all_sensor_data();
                break;
            case 2: // one sensor data point
                get_sensor_data();
                break;
            case 3: // archived sensor data
                get_archived_sensor_data();
                break;
            case 4: // one parameter
                get_one_param();
                break;
            case 5: // one parameter
                get_all_params();
                break;
            default:
                break;
        }
        return (spark_register[0] == '\0' ? -1 : 1);
    }
}

int run_command(String msg) {
    parse_spark_command(msg);

    switch (spark_command.code) {
        case 0: // write serial number
            return write_serial_number();
        case 1: // initialize device
            return initialize_device();
        case 2: // run assay
            return run_brevitest();
        case 3: // collect sensor data
            return recollect_sensor_data();
        case 4: // collect sensor data
            return change_param();
        case 5: // collect sensor data
            return reset_params();
        default:
            return -1;
    }
    return -1;
}


//
//
//  UTILITY
//
//

void parse_spark_request(String msg) {
    int len = msg.length();
    msg.toCharArray(spark_request.arg, len + 1);

    strncpy(spark_request.uuid, spark_request.arg, UUID_LENGTH);
    spark_request.uuid[UUID_LENGTH] = '\0';
    spark_request.code = extract_int_from_argument(spark_request.arg, UUID_LENGTH, REQUEST_CODE_LENGTH);
    len -= UUID_LENGTH + REQUEST_CODE_LENGTH;
    strncpy(spark_request.param, &spark_request.arg[UUID_LENGTH + REQUEST_CODE_LENGTH], len);
    spark_request.param[len] = '\0';

    spark_request.pending = true;
}

void parse_spark_command(String msg) {
    int len = msg.length();
    msg.toCharArray(spark_command.arg, len + 1);

    spark_command.code = extract_int_from_argument(spark_command.arg, 0, COMMAND_CODE_LENGTH);
    len -= COMMAND_CODE_LENGTH;
    strncpy(spark_command.param, &spark_command.arg[COMMAND_CODE_LENGTH], len);
    spark_command.param[len] = '\0';
}

void write_default_params() {
    Param reset;

    EEPROM.write(0, 0xA1);
    flash->write(&reset, 0, sizeof(reset));
}

void read_params() {
    flash->read(&brevitest, 0, sizeof(brevitest));
}

void dump_params() {
    int *ptr;

    ptr = (int *) &brevitest;
    for (int i = 0; i < NUMBER_OF_PARAMS; i += 1) {
        Serial.println(*ptr++);
    }
}

void load_params() {
    char c = EEPROM.read(0);
    if (c != 0xA1) {
        Serial.println("Writing default params");
        write_default_params();
    }
    else {
        Serial.println("Reading params");
        read_params();
    }
}

int extract_int_from_argument(char *str, int pos, int len) {
    char buf[12];

    strncpy(buf, &str[pos], len);
    return atoi(buf);
}


//
//
//  SETUP
//
//

void setup() {
    Spark.function("runcommand", run_command);
    Spark.function("requestdata", request_data);
    Spark.variable("register", spark_register, STRING);
    Spark.variable("status", spark_status, STRING);

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
//    while (!Serial.available()) {
//        Spark.process();
//    }
    Serial.println(EEPROM.read(0), HEX);

    flash = Devices::createWearLevelErase();

    load_params();

    device_ready = false;
    init_device = false;
    run_assay = false;

    spark_register[0] = '\0';

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
