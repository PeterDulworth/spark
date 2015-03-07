#include "TCS34725.h"
#include "SoftI2CMaster.h"
 #include "flashee-eeprom.h"
    using namespace Flashee;

#include "main.h"

/////////////////////////////////////////////////////////////
//                                                         //
//                        UTLITY                           //
//                                                         //
/////////////////////////////////////////////////////////////

int extract_int_from_string(char *str, int pos, int len) {
    char buf[12];

    strncpy(buf, &str[pos], len);
    return atoi(buf);
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        SOLENOID                         //
//                                                         //
/////////////////////////////////////////////////////////////

void solenoid_energize(int duration) {
    Spark.process();

    if (cancel_process) {
        return;
    }

    analogWrite(pinSolenoid, brevitest.solenoid_surge_power);
    delay(brevitest.solenoid_surge_period_ms);
    analogWrite(pinSolenoid, brevitest.solenoid_sustain_power);
    delay(duration - brevitest.solenoid_surge_period_ms);
    analogWrite(pinSolenoid, 0);
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        STEPPER                          //
//                                                         //
/////////////////////////////////////////////////////////////

void move_steps(long steps, int step_delay){
    //rotate a specific number of steps - negative for reverse movement

    wake_stepper();

    int dir = (steps > 0)? LOW:HIGH;
    steps = abs(steps);

    digitalWrite(pinStepperDir,dir);

    for(long i = 0; i < steps; i += 1) {
        if (cancel_process) {
            break;
        }
        if ((dir == HIGH) && (digitalRead(pinLimitSwitch) == LOW)) {
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

void reset_stage() {
    STATUS("Resetting device");
    move_steps(SPARK_RESET_STAGE_STEPS, brevitest.step_delay_us);
}

void move_to_calibration_point() {
    int calibration_steps;

    calibration_steps = EEPROM.read(EEPROM_ADDR_CALIBRATION_STEPS);
    calibration_steps <<= 8;
    calibration_steps += EEPROM.read(EEPROM_ADDR_CALIBRATION_STEPS + 1);

    CANCELLABLE(reset_stage();)
    delay(500);
    CANCELLABLE(move_steps(calibration_steps, brevitest.step_delay_us);)
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        SENSORS                          //
//                                                         //
/////////////////////////////////////////////////////////////

void init_sensor(TCS34725 *sensor, int sdaPin, int sclPin) {
    uint8_t it, gain;

    it = brevitest.sensor_params >> 8;
    gain = brevitest.sensor_params & 0x00FF;
    init_sensor_with_params(sensor, sdaPin, sclPin, it, gain);
}

void init_sensor_with_params(TCS34725 *sensor, int sdaPin, int sclPin, uint8_t it, uint8_t gain) {
    *sensor = TCS34725((tcs34725IntegrationTime_t) it, (tcs34725Gain_t) gain, sdaPin, sclPin);

    if (sensor->begin()) {
        sensor->enable();
    }
    else {
        ERROR_MESSAGE("Sensor not found");
    }
}

void read_sensor(TCS34725 *sensor, char sensor_code, int sample_number, int number_of_samples) {
    BrevitestSensorRecord *sample;
    int index = 2 * (test_sensor_sample_count + sample_number) + (sensor_code == 'A' ? 0 : 1);

    sample = &test_record.sensor_reading[index];
    sample->sensor_code = sensor_code;
    sample->sample_number = sample_number;
    sample->reading_time = Time.now();

    Spark.process();

    STATUS("Reading %s sensor (%d of %d)", (sensor_code == 'A' ? "Assay" : "Control"), sample_number + 1, number_of_samples);

    sensor->getRawData(&sample->red, &sample->green, &sample->blue, &sample->clear);
}

void sample_sensors(int number_of_samples, int delay_between_samples) {
    STATUS("Collecting sensor data");
    for (int i = 0; i < number_of_samples; i += 1) {
        if (cancel_process) {
            return;
        }
        read_sensor(&tcsAssay, 'A', i, number_of_samples);
        read_sensor(&tcsControl, 'C', i, number_of_samples);
        delay(delay_between_samples);
    }
    test_sensor_sample_count += number_of_samples;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                    FLASH AND PARAMS                     //
//                                                         //
/////////////////////////////////////////////////////////////

int get_flash_test_address(int num) {
    return TEST_RECORD_START_ADDR + num * TEST_RECORD_LENGTH;
}

/*int get_flash_test_address_by_uuid(char *uuid) {
    int i;
    char flash_uuid[UUID_LENGTH];
    int count = get_flash_test_record_count();
    int index = TEST_RECORD_START_ADDR + TEST_RECORD_UUID_OFFSET;

    for (i = 0; i < count; i += 1) {
        flash->read(flash_uuid, index, UUID_LENGTH);
        if (strncmp(uuid, flash_uuid, UUID_LENGTH) == 0) {
            return (index - TEST_RECORD_UUID_OFFSET);
        }
        index += TEST_RECORD_LENGTH;
    }

    return -1;
}*/

int get_flash_test_record_count() {
    int count;
    flash->read(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    count %= FLASH_RECORD_CAPACITY; // circular buffer
    return count;
}

void write_test_record_to_flash(int ref_time) {
    STATUS("Writing sensor data to flash memory");

    int test_addr;

    // build test record
    test_record.num = get_flash_test_record_count();
    test_record.start_time = ref_time;
    memcpy(test_record.uuid, test_uuid, UUID_LENGTH);
    test_record.uuid[UUID_LENGTH] = '\0';
    memcpy(&test_record.param, &brevitest, PARAM_TOTAL_LENGTH);
    test_record.BCODE_version = 1;
    test_record.num_samples = test_sensor_sample_count;
    test_record.BCODE_length = BCODE_length;

    // copy BCODE and sensor data
    strncpy(test_record.BCODE, test_record.BCODE, BCODE_length);

    // write test record
    test_addr = get_flash_test_address(test_record.num);
    flash->write(&test_record, test_addr, TEST_RECORD_LENGTH);

    // update record count
    test_record.num++;
    flash->write(&test_record.num, TEST_NUMBER_OF_RECORDS_ADDR, 4);
}

void write_default_params() {
    Param reset;
    flash->write(&reset, 0, sizeof(reset));
}

void read_params() {
    flash->read(&brevitest, 0, sizeof(brevitest));
}

void dump_params() {
    int *ptr;

    ptr = (int *) &brevitest;
    for (int i = 0; i < PARAM_COUNT; i += 1) {
        Serial.println(*ptr++);
    }
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        REQUESTS                         //
//                                                         //
/////////////////////////////////////////////////////////////

int get_serial_number() {
    STATUS("Retrieving device serial number");
    spark_register[SERIAL_NUMBER_LENGTH] = '\0';
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        spark_register[i] = (char) EEPROM.read(EEPROM_ADDR_SERIAL_NUMBER + i);
    }
    return 0;
}

int get_test_record() {
    // spark_request.param is num

    STATUS("Retrieving test record");

    BrevitestSensorRecord *reading;
    int addr, count, i, len, num;

    count = get_flash_test_record_count();
    if (count <= 0) { // no records
      return -1;
    }

    num = atoi(spark_request.param);
    if (spark_request.index >= (FLASH_OVERFLOW_ADDRESS - TEST_RECORD_LENGTH)) { // overflow
      return -1;
    }

    if (spark_request.index == 0) {  // initial request
        addr = get_flash_test_address(num);
        if (addr == -1) {
          return -1;
        }

        test_num = num;

        flash->read(&test_record, addr, TEST_RECORD_LENGTH);

        len = snprintf(test_string, TEST_RECORD_STRING_LENGTH, \
                "%3d\t%11d\t%32s\t%3d\t%3d\t%4d\n%5d\t%5d\t%5d\t%3d\t%5d\t%3d\t%11d\t%6d\t%3d\t%6d\n%s\n", \
                test_record.num, test_record.start_time, test_record.uuid, test_record.num_samples, test_record.BCODE_version, test_record.BCODE_length, \
                test_record.param.step_delay_us, test_record.param.stepper_wifi_ping_rate, test_record.param.stepper_wake_delay_ms, \
                test_record.param.solenoid_surge_power, test_record.param.solenoid_surge_period_ms, test_record.param.solenoid_sustain_power, \
                test_record.param.sensor_params, test_record.param.sensor_ms_between_samples, test_record.param.sensor_led_power, \
                test_record.param.sensor_led_warmup_ms, test_record.BCODE);

        for (i = 0; i < 2 * test_record.num_samples; i += 1) {
            reading = &test_record.sensor_reading[i];
            len += snprintf(&test_string[len], TEST_RECORD_STRING_LENGTH - len, \
                    "%c\t%2d\t%11d\t%5d\t%5d\t%5d\t%5d\n", \
                    reading->sensor_code, reading->sample_number, reading->reading_time, \
                    reading->clear, reading->red, reading->green, reading->blue);
        }
        test_string[--len] = '\0';
        test_string_length = len;
    }
    else {  // continuation request
        if (num != test_num) {
            return -1;
        }
    }

    len = test_string_length - spark_request.index;
    len = (len < SPARK_REGISTER_SIZE ? len : SPARK_REGISTER_SIZE);
    memcpy(spark_register, &test_string[spark_request.index], len);
    spark_register[len] = '\0';
    spark_request.index += len;
    if (spark_request.index >= test_string_length) {
        spark_request.index = 0;
    }

    return spark_request.index;
}

int get_one_param() {
    int value;

    int num = extract_int_from_string(spark_request.param, 0, strlen(spark_request.param));
    flash->read(&value, num, 4);
    sprintf(spark_register, "%d", value);

    return 0;
}

int get_all_params() {
    int value, len;
    int offset = 0;
    int index = 0;

    for (int i = 0; i < PARAM_TOTAL_LENGTH; i += 4) {
        flash->read(&value, i + offset, 4);
        len = sprintf(&spark_register[index], "%d,", value);
        index += len;
    }
    spark_register[--index] = '\0';

    return 0;
}

//
//

int parse_spark_request(String msg) {
    int len = msg.length();
    int param_length = len - REQUEST_PARAM_INDEX;

    msg.toCharArray(spark_request.arg, len + 1);

    spark_request.index = extract_int_from_string(spark_request.arg, REQUEST_INDEX_INDEX, REQUEST_INDEX_LENGTH);

    if (spark_request.index == 0) { // first request
        strncpy(spark_request.uuid, spark_request.arg, UUID_LENGTH);
        spark_request.uuid[UUID_LENGTH] = '\0';
    }
    else {
        if (strncmp(spark_request.arg, spark_request.uuid, UUID_LENGTH) != 0) {
            ERROR_MESSAGE("Register uuid mismatch");
            return -1;
        }
        if (spark_request.index == 999999) {
            spark_register[0] = '\0';
            return 0;
        }
    }

    spark_request.code = extract_int_from_string(spark_request.arg, REQUEST_CODE_INDEX, REQUEST_CODE_LENGTH);
    strncpy(spark_request.param, &spark_request.arg[REQUEST_PARAM_INDEX], param_length);
    spark_request.param[param_length] = '\0';

    return 1;
}

int request_data(String msg) {
    if (parse_spark_request(msg) != -1) {
        switch (spark_request.code) {
            case 0: // serial_number
                return get_serial_number();
            case 1: // get test record
                return get_test_record();
            case 2: // get params
                return get_all_params();
            case 3: // one parameter
                return get_one_param();
        }
    }

    return -1;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        COMMANDS                         //
//                                                         //
/////////////////////////////////////////////////////////////

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
    if (BCODE_length == 0) {
        ERROR_MESSAGE("BCODE not found. Assay not started.");
        return -1;
    }
    if (memcmp(BCODE_uuid, &spark_command.param, UUID_LENGTH) != 0) {
        ERROR_MESSAGE("BCODE uuid doesn't match test uuid. Assay not started.");
        return -1;
    }

    memcpy(test_uuid, &spark_command.param, UUID_LENGTH);
    test_uuid[UUID_LENGTH] = '\0';
    run_assay = true;
    return 1;
}

int collect_sensor_samples() {
    collect_sensor_data = true;
    return 1;
}

int change_param() {
    int param_index;
    int value;

    STATUS("Changing parameter value");
    param_index = extract_int_from_string(spark_command.param, 0, PARAM_CODE_LENGTH);
    if (param_index >= PARAM_TOTAL_LENGTH) {
        ERROR_MESSAGE("Parameter index out of range");
        return -1;
    }
    value = extract_int_from_string(spark_command.param, PARAM_CODE_LENGTH, strlen(spark_command.param));
    flash->write(&value, param_index, 4);
    read_params();

    return value;
}

int reset_params() {
    STATUS("Resetting parameters to default values");

    write_default_params();
    read_params();

    return 1;
}

int erase_archived_data() {
    STATUS("Erase archived data");
    int count = 0;
    flash->write(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    return 1;
}

int dump_archive() {
    STATUS("Dumping memory to serial port");
    int i;
    char buf[64];

    for (i = 0; i < FLASH_OVERFLOW_ADDRESS; i += 64) {
        flash->read(buf, i, 64);
        Serial.print(buf);
    }
    Serial.println();
    return 1;
}

int get_archive_size() {
    int count;
    flash->read(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    return count;
}

int get_firmware_version() {
    int version = EEPROM.read(0);
    return version;
}

int cancel_current_process() {
    cancel_process = true;
    return 1;
}

int receive_BCODE() {
    int num;

    STATUS("Receiving BCODE payload");

    num = extract_int_from_string(spark_command.param, 0, BCODE_NUM_LENGTH);
    if (BCODE_count != num) {
        ERROR_MESSAGE("BCODE index mismatch");
        ERROR_MESSAGE(BCODE_count);
        ERROR_MESSAGE(num);
        BCODE_length = 0;
        return -1;
    }
    if (num == 0) { // first packet, contains number of packets (not including this packet)
        BCODE_packets = extract_int_from_string(spark_command.param, BCODE_NUM_LENGTH, BCODE_LEN_LENGTH);
        memcpy(BCODE_uuid, &spark_command.param[BCODE_UUID_INDEX], UUID_LENGTH);
        BCODE_index = 0;
        BCODE_length = 0;
    }
    else { // payload packet, contains payload count, packet length, uuid, and payload
        if (num <= BCODE_packets) {
            if (memcmp(BCODE_uuid, &spark_command.param[BCODE_UUID_INDEX], UUID_LENGTH) != 0) {
                ERROR_MESSAGE("BCODE uuid mismatch");
                BCODE_length = 0;
                return -1;
            }
            BCODE_length = extract_int_from_string(spark_command.param, BCODE_NUM_LENGTH, BCODE_LEN_LENGTH);
            if ((BCODE_index + BCODE_length) > BCODE_CAPACITY) {
                ERROR_MESSAGE("BCODE buffer overflow");
                BCODE_length = 0;
                return -1;
            }
            memcpy(&test_record.BCODE[BCODE_index], &spark_command.param[BCODE_PAYLOAD_INDEX], BCODE_length);
            BCODE_index += BCODE_length;
        }
        else {
            ERROR_MESSAGE("BCODE packet overflow");
            BCODE_length = 0;
            return -1;
        }
    }

    if (num < BCODE_packets) {
        BCODE_count++;
    }
    else { // last packet
        BCODE_length = BCODE_index;
        test_record.BCODE[BCODE_length] = '\0';
        BCODE_count = 0;
        BCODE_packets = 0;
        BCODE_index = 0;
    }

    return num;
}

int set_and_move_to_calibration_point() {
    int calibration_steps;
    uint8_t lsb, msb;

    calibration_steps = extract_int_from_string(spark_command.param, 0, PARAM_CODE_LENGTH);

    msb = (uint8_t) (calibration_steps >> 8);
    lsb = (uint8_t) (calibration_steps & 0x0F);
    EEPROM.write(EEPROM_ADDR_CALIBRATION_STEPS, msb);
    EEPROM.write(EEPROM_ADDR_CALIBRATION_STEPS + 1, lsb);

    move_to_calibration_point();

    return 1;
}

//
//

void parse_spark_command(String msg) {
    int len = msg.length();
    msg.toCharArray(spark_command.arg, len + 1);

    spark_command.code = extract_int_from_string(spark_command.arg, 0, COMMAND_CODE_LENGTH);
    len -= COMMAND_CODE_LENGTH;
    strncpy(spark_command.param, &spark_command.arg[COMMAND_CODE_LENGTH], len);
    spark_command.param[len] = '\0';
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
        case 3: // collect sensor samples
            return collect_sensor_samples();
        case 4: // change device parameter
            return change_param();
        case 5: // reset device parameters to default
            return reset_params();
        case 6: // factor reset
            return erase_archived_data();
        case 7: // dump archive to serial port
            return dump_archive();
        case 8: // dump archive to serial port
            return get_archive_size();
        case 9: // get current firmware version number
            return get_firmware_version();
        case 10: // cancel process
            return cancel_current_process();
        case 11: // receive BCODE string
            return receive_BCODE();
        case 12: // device ready
            return (device_ready ? 1 : -1);
        case 13: // set and move to calibration point
            return set_and_move_to_calibration_point();
        default:
            return -1;
    }
    return -1;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                          BCODE                          //
//                                                         //
/////////////////////////////////////////////////////////////

int get_BCODE_token(int index, int *token) {
    int i;

    if (cancel_process) {
        return index;
    }

    if (test_record.BCODE[index] == '\0') { // end of string
        return index; // return end of string location
    }

    if (test_record.BCODE[index] == '\n') { // command has no parameter
        return index + 1; // skip past parameter
    }

    // there is a parameter to extract
    i = index;
    while (i < BCODE_CAPACITY) {
        if (test_record.BCODE[i] == '\0') {
            *token = extract_int_from_string(test_record.BCODE, index, (i - index));
            return i; // return end of string location
        }
        if ((test_record.BCODE[i] == '\n') || (test_record.BCODE[i] == ',')) {
            *token = extract_int_from_string(test_record.BCODE, index, (i - index));
            i++; // skip past parameter
            return i;
        }

        i++;
    }

    return i;
}

int process_one_BCODE_command(int cmd, int index) {
    int i, param1, param2, start_index;

    if (cancel_process) {
        return index;
    }

    switch(cmd) {
        case 0: // Start Assay(integration time, gain)
            test_start_time = Time.now();
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);

            init_sensor_with_params(&tcsAssay, pinAssaySDA, pinAssaySCL, (uint8_t) param1, (uint8_t) param2);
            init_sensor_with_params(&tcsControl, pinControlSDA, pinControlSCL, (uint8_t) param1, (uint8_t) param2);
            test_sensor_sample_count = 0;
            break;
        case 1: // Delay(milliseconds)
            index = get_BCODE_token(index, &param1);
            delay(param1);
            break;
        case 2: // Move(number of steps, step delay)
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);
            move_steps(param1, param2);
            break;
        case 3: // Solenoid on(milliseconds)
            index = get_BCODE_token(index, &param1);
            solenoid_energize(param1);
            break;
        case 4: // Device LED on
            analogWrite(pinDeviceLED, 255);
            break;
        case 5: // Device LED off
            analogWrite(pinDeviceLED, 0);
            break;
        case 6: // Device LED blink(milliseconds)
            index = get_BCODE_token(index, &param1);
            analogWrite(pinDeviceLED, 255);
            delay(param1);
            analogWrite(pinDeviceLED, 0);
            break;
        case 7: // Sensor LED on(power)
            index = get_BCODE_token(index, &param1);
            analogWrite(pinSensorLED, param1);
            break;
        case 8: // Sensor LED off
            analogWrite(pinSensorLED, 0);
            break;
        case 9: // Read sensors(number of samples)
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);
            param1 = (param1 < SENSOR_SAMPLE_CAPACITY ? param1 : SENSOR_SAMPLE_CAPACITY);
            param1 = (param1 > 0 ? param1 : 0);
            sample_sensors(param1, param2);
            break;
        case 10: // Read QR code
            Serial.println("QR Code not implemented");
            break;
        case 11: // Disable sensor
            tcsAssay.disable();
            tcsControl.disable();
            break;
        case 12: // Repeat begin(number of iterations)
            index = get_BCODE_token(index, &param1);

            start_index = index;
            for (i = 0; i < param1; i += 1) {
                if (cancel_process) {
                    break;
                }
                index = process_BCODE(start_index);
            }
            break;
        case 13: // Repeat end
            return -index;
            break;
        case 14: // Status(message length, message text)
            index = get_BCODE_token(index, &param1);
            param1++;
            snprintf(spark_status, param1, &test_record.BCODE[index]);
            index += param1;
            break;
        case 99: // Finish Assay
            STATUS("Storing assay results and resetting device");
            write_test_record_to_flash(test_start_time);
            reset_stage();
            break;
    }

    return index;
}

int process_BCODE(int start_index) {
    int cmd, index;

    Spark.process();
    index = get_BCODE_token(start_index, &cmd);
    if ((start_index == 0) && (cmd != 0)) { // first command
        cancel_process = true;
        ERROR_MESSAGE("First BCODE command must be Start Assay. Test cancelled.");
        return -1;
    }
    else {
        index = process_one_BCODE_command(cmd, index);
    }

    while ((cmd != 99) && (index > 0) && !cancel_process) {
        Spark.process();
        index = get_BCODE_token(index, &cmd);
        index = process_one_BCODE_command(cmd, index);
    };

    return (index > 0 ? index : -index);
}

/////////////////////////////////////////////////////////////
//                                                         //
//                          SETUP                          //
//                                                         //
/////////////////////////////////////////////////////////////

void setup() {
    int count;

    Spark.function("runcommand", run_command);
    Spark.function("requestdata", request_data);
    Spark.variable("register", spark_register, STRING);
    Spark.variable("status", spark_status, STRING);
    Spark.variable("testrunning", test_uuid, STRING);

    pinMode(pinSolenoid, OUTPUT);
    pinMode(pinStepperStep, OUTPUT);
    pinMode(pinStepperDir, OUTPUT);
    pinMode(pinStepperSleep, OUTPUT);
    pinMode(pinLimitSwitch, INPUT_PULLUP);
    pinMode(pinSensorLED, OUTPUT);
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

    if (EEPROM.read(0) != FIRMWARE_VERSION) {  // check for current firmware version
        EEPROM.write(0, FIRMWARE_VERSION);
    }

    flash = Devices::createWearLevelErase();

    read_params();

    flash->read(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    if (count == -1) { // brand new core
        count = 0;
        flash->write(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
        write_default_params();
    }

    device_ready = false;
    init_device = false;
    run_assay = false;
    collect_sensor_data = false;
    cancel_process = false;

    BCODE_count = 0;
    BCODE_length = 0;

    spark_register[0] = '\0';

    STATUS("Setup complete");
}

/////////////////////////////////////////////////////////////
//                                                         //
//                           LOOP                          //
//                                                         //
/////////////////////////////////////////////////////////////

void do_initialize_device() {
    STATUS("Initializing device");

    analogWrite(pinSolenoid, 0);
    analogWrite(pinSensorLED, 0);

    move_to_calibration_point();

    device_ready = true;
    init_device = false;

    STATUS("Device initialized and ready to run assay");
}

void do_run_test() {
    device_ready = false;
    STATUS("Running assay...");

    process_BCODE(0);

    STATUS("Assay complete.");
    run_assay = false;
    test_uuid[0] = '\0';
}

void do_sensor_data_collection() {
    STATUS("Initializing sensors");
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
    analogWrite(pinSensorLED, brevitest.sensor_led_power);

    STATUS("Warming up sensor LEDs");
    delay(brevitest.sensor_led_warmup_ms);

    int t = Time.now();
    test_sensor_sample_count = 0;
    CANCELLABLE(sample_sensors(SENSOR_COLLECT_SAMPLES, brevitest.sensor_ms_between_samples);)
    CANCELLABLE(write_test_record_to_flash(t);)

    analogWrite(pinSensorLED, 0);
    tcsAssay.disable();
    tcsControl.disable();
    collect_sensor_data = false;
    STATUS("Sensor data collection complete");
}

void loop(){
    if (init_device) {
        do_initialize_device();
    }

    if (device_ready && run_assay) {
        do_run_test();
    }

    if (collect_sensor_data) {
        do_sensor_data_collection();
    }

    if (cancel_process) {
        STATUS("Process cancelled");
        cancel_process = false;
    }
}
