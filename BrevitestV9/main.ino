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

    it = ((int) brevitest.sensor_params) & 0x00FF;
    gain = ((int) brevitest.sensor_params) >> 8;
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

void read_one_sensor(TCS34725 *sensor, char sensor_code, int sample_number) {
    BrevitestSensorSampleRecord *sample;

    if (sensor_code == 'A') {
        sample = &assay_buffer[sample_number];
    }
    else {
        sample = &control_buffer[sample_number];
    }
    sample->sensor_code = sensor_code;
    sample->sample_number = sample_number;
    sample->sample_time = Time.now();

    Spark.process();

    sensor->getRawData(&sample->red, &sample->green, &sample->blue, &sample->clear);
}

void convert_samples_to_reading(int reading_code, char sensor_code) {
    int i, j, k;
    int red, green, blue, clear;
    int index[SENSOR_NUMBER_OF_SAMPLES];
    BrevitestSensorSampleRecord *buffer;
    BrevitestSensorRecord *reading;

    if (sensor_code == 'A') {
        buffer = assay_buffer;
        if (reading_code == 0) {
            reading = &test_record.sensor_reading_initial_assay;
        }
        else {
            reading = &test_record.sensor_reading_final_assay;
        }
    }
    else {
        buffer = control_buffer;
        if (reading_code == 0) {
            reading = &test_record.sensor_reading_initial_control;
        }
        else {
            reading = &test_record.sensor_reading_final_control;
        }
    }

    index[0] = 0;
    for (i = 1; i < SENSOR_NUMBER_OF_SAMPLES; i += 1) {
        for (j = 0; j < i; j += 1) {
            if (buffer[i].clear < buffer[index[j]].clear) {
                for (k = i; k > j; k -= 1) {
                    index[k] = index[k - 1];
                }
                index[j] = i;
                break;
            }
            if (j == i - 1) {
                index[i] = i;
            }
        }
    }

    reading->red_norm = reading->green_norm = reading->blue_norm = 0;
    for (j = 1; j < SENSOR_NUMBER_OF_SAMPLES - 1; j += 1) {
        i = index[j];
        reading->red_norm += (10000 * ((int) buffer[i].clear - (int) buffer[i].red)) / (int) buffer[i].clear;
        reading->green_norm += (10000 * ((int) buffer[i].clear - (int) buffer[i].green)) / (int) buffer[i].clear;
        reading->blue_norm += (10000 * ((int) buffer[i].clear - (int) buffer[i].blue)) / (int) buffer[i].clear;
    }
    reading->red_norm /= SENSOR_NUMBER_OF_SAMPLES - 2;
    reading->green_norm /= SENSOR_NUMBER_OF_SAMPLES - 2;
    reading->blue_norm /= SENSOR_NUMBER_OF_SAMPLES - 2;

    reading->sensor_code = sensor_code;
    reading->number = reading_code;
    reading->start_time = buffer[0].sample_time;
}

void read_sensors(int reading_code) { // 0 -> initial baseline, 1 -> assay
    for (int i = 0; i < SENSOR_NUMBER_OF_SAMPLES; i += 1) {
        if (cancel_process) {
            return;
        }
        read_one_sensor(&tcsAssay, 'A', i);
        read_one_sensor(&tcsControl, 'C', i);
        delay(SENSOR_DELAY_BETWEEN_SAMPLES);
        if (reading_code == 0) {
            update_progress("Taking baseline readings", SENSOR_DELAY_BETWEEN_SAMPLES);
        }
        else {
            update_progress("Reading test results", SENSOR_DELAY_BETWEEN_SAMPLES);
        }
    }

    convert_samples_to_reading(reading_code, 'A');
    convert_samples_to_reading(reading_code, 'C');
}

/////////////////////////////////////////////////////////////
//                                                         //
//                    FLASH AND PARAMS                     //
//                                                         //
/////////////////////////////////////////////////////////////

int get_flash_test_address(int num) {
    if (num <= get_flash_test_record_count()) {
        return TEST_RECORD_START_ADDR + num * TEST_RECORD_LENGTH;
    }
    else {
        return -1;
    }
}

int get_flash_test_address_by_uuid(char *uuid) {
    int i, index;

    int count = get_flash_test_record_count();
    if (count <= 0) { // no records
      return -1;
    }
    index = TEST_RECORD_START_ADDR + TEST_RECORD_UUID_OFFSET;
    for (i = 0; i < count; i += 1) {
        flash->read(test_record.uuid, index, UUID_LENGTH);

        if (strncmp(uuid, test_record.uuid, UUID_LENGTH) == 0) {
            index -= TEST_RECORD_UUID_OFFSET;
            return index;
        }
        index += TEST_RECORD_LENGTH;
    }

    return -1;
}

int get_flash_test_record_count() {
    int count;
    flash->read(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    count %= FLASH_RECORD_CAPACITY; // circular buffer
    return count;
}

void write_test_record_to_flash() {
    int test_addr;
    // build test record
    test_record.num = get_flash_test_record_count();

    strncpy(test_record.uuid, test_uuid, UUID_LENGTH);
    memcpy(&test_record.param, &brevitest, PARAM_TOTAL_LENGTH);
    test_record.BCODE_version = 1;
    test_record.BCODE_length = BCODE_length;

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
    spark_register[SERIAL_NUMBER_LENGTH] = '\0';
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        spark_register[i] = (char) EEPROM.read(EEPROM_ADDR_SERIAL_NUMBER + i);
    }
    return 0;
}

int process_test_record(int addr) {
    BrevitestSensorRecord *reading;
    int finish, i, len, start;
    int packet_max, num_readings, num_readings_per_packet, packet_num;

    flash->read(&test_record, addr, TEST_RECORD_LENGTH);

    len = snprintf(spark_register, SPARK_REGISTER_SIZE, \
        "%3d\t%11d\t%11d\t%24s\t%3d\t%4d\t%3d\t%3d\n%5d\t%5d\t%5d\t%3d\t%5d\t%3d\t%11d\t%6d\t%3d\t%6d\n%c\t%2d\t%11d\t%5d\t%5d\t%5d\n%c\t%2d\t%11d\t%5d\t%5d\t%5d\n%c\t%2d\t%11d\t%5d\t%5d\t%5d\n%c\t%2d\t%11d\t%5d\t%5d\t%5d\n", \
        test_record.num, test_record.start_time, test_record.finish_time, test_record.uuid, test_record.BCODE_version, test_record.BCODE_length, \
        test_record.integration_time, test_record.gain, \
        test_record.param.step_delay_us, test_record.param.stepper_wifi_ping_rate, test_record.param.stepper_wake_delay_ms, \
        test_record.param.solenoid_surge_power, test_record.param.solenoid_surge_period_ms, test_record.param.solenoid_sustain_power, \
        test_record.param.sensor_params, test_record.param.sensor_ms_between_samples, test_record.param.sensor_led_power, \
        test_record.param.sensor_led_warmup_ms, \
        test_record.sensor_reading_initial_assay.sensor_code, test_record.sensor_reading_initial_assay.number, \
        test_record.sensor_reading_initial_assay.start_time, test_record.sensor_reading_initial_assay.red_norm, \
        test_record.sensor_reading_initial_assay.green_norm, test_record.sensor_reading_initial_assay.blue_norm, \
        test_record.sensor_reading_initial_control.sensor_code, test_record.sensor_reading_initial_control.number, \
        test_record.sensor_reading_initial_control.start_time, test_record.sensor_reading_initial_control.red_norm, \
        test_record.sensor_reading_initial_assay.green_norm, test_record.sensor_reading_initial_assay.blue_norm, \
        test_record.sensor_reading_final_assay.sensor_code, test_record.sensor_reading_final_assay.number, \
        test_record.sensor_reading_final_assay.start_time, test_record.sensor_reading_final_assay.red_norm, \
        test_record.sensor_reading_initial_assay.green_norm, test_record.sensor_reading_initial_assay.blue_norm, \
        test_record.sensor_reading_final_control.sensor_code, test_record.sensor_reading_final_control.number, \
        test_record.sensor_reading_final_control.start_time, test_record.sensor_reading_final_control.red_norm, \
        test_record.sensor_reading_initial_assay.green_norm, test_record.sensor_reading_initial_assay.blue_norm);

    spark_register[len] = '\0';
    return 0;
}

int get_test_record() {
    // spark_request.param is num
    int addr;

    if (spark_request.index == 0) {  // initial request
        test_num = atoi(spark_request.param);
    }
    else {  // continuation request
        if (atoi(spark_request.param) != test_num) {
            return -3;
        }
    }

    addr = get_flash_test_address(test_num);
    if (addr == -1) {
      return -2;
    }

    return process_test_record(addr);
}

int get_test_record_by_uuid() {
    // spark_request.param is num
    int addr;

    if (spark_request.index == 0) {  // initial request
        strncpy(test_record.uuid, spark_request.uuid, UUID_LENGTH);
    }
    else {  // continuation request
        if (strncmp(spark_request.uuid, test_record.uuid, UUID_LENGTH) != 0) {
            return -3;
        }
    }

    addr = get_flash_test_address_by_uuid(spark_request.uuid);
    if (addr == -1) {
      return -2;
    }

    return process_test_record(addr);
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
    if (parse_spark_request(msg) > 0) {
        switch (spark_request.code) {
            case 0: // serial_number
                return get_serial_number();
            case 1: // get test record
                return get_test_record();
            case 2: // get test record
                return get_test_record_by_uuid();
            case 3: // get params
                return get_all_params();
            case 4: // one parameter
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
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        EEPROM.write(EEPROM_ADDR_SERIAL_NUMBER + i, spark_command.param[i]);
    }
    return 1;
}

int initialize_device() {
    init_device = !test_in_progress;
    return 1;
}

int run_brevitest() {
    if (!device_ready) {
        ERROR_MESSAGE(-10);
        return -1;
    }
    if (test_in_progress) {
        ERROR_MESSAGE(-11);
        return -1;
    }
    if (BCODE_length == 0) {
        ERROR_MESSAGE(-12);
        return -1;
    }
    if (memcmp(BCODE_uuid, &spark_command.param, UUID_LENGTH) != 0) {
        ERROR_MESSAGE(-13);
        return -1;
    }

    strncpy(test_uuid, spark_command.param, UUID_LENGTH);
    test_uuid[UUID_LENGTH] = '\0';
    test_duration = 1000 * extract_int_from_string(spark_command.param, UUID_LENGTH, TEST_DURATION_LENGTH);
    start_test = true;
    update_progress("Starting test", 0);
    return 1;
}

int collect_sensor_samples() {
    collect_sensor_data = true;
    return 1;
}

int change_param() {
    int param_index;
    int value;

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
    write_default_params();
    read_params();

    return 1;
}

int erase_archived_data() {
    Serial.println("Erasing archived data");
    flash->eraseAll();
    int count = 0;
    flash->write(&count, TEST_NUMBER_OF_RECORDS_ADDR, 4);
    reset_params();
    return 1;
}

int dump_archive() {
    int i, param_index;
    char buf[64];

    param_index = atoi(spark_command.param);
    for (i = 0; i < param_index; i += 64) {
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

    STATUS("Receiving BCODE");

    num = extract_int_from_string(spark_command.param, 0, BCODE_NUM_LENGTH);
    if (BCODE_count != num) {
        ERROR_MESSAGE("BCODE index mismatch");
        ERROR_MESSAGE(spark_command.param);
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
                ERROR_MESSAGE(-20);
                BCODE_length = 0;
                return -1;
            }
            BCODE_length = extract_int_from_string(spark_command.param, BCODE_NUM_LENGTH, BCODE_LEN_LENGTH);
            if ((BCODE_index + BCODE_length) > BCODE_CAPACITY) {
                ERROR_MESSAGE(-21);
                BCODE_length = 0;
                return -1;
            }
            memcpy(&test_record.BCODE[BCODE_index], &spark_command.param[BCODE_PAYLOAD_INDEX], BCODE_length);
            BCODE_index += BCODE_length;
        }
        else {
            ERROR_MESSAGE(-22);
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
        case 2: // run test
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

void update_progress(char *message, int duration) {
    unsigned long now = millis();

    if (!cancel_process) {
        if (duration == 0) {
            test_progress = 0;
            test_percent_complete = 0;
        }
        else if (duration < 0) {
            test_progress = test_duration;
            test_percent_complete = 100;
        }
        else {
            test_progress += duration;
            test_percent_complete = 100 * test_progress / test_duration;
        }

        STATUS("%s\n%s\n%d", message, test_uuid, test_percent_complete);
        Serial.println(spark_status);
        if (now - test_last_progress_update < 1000) {
            return;
        }
        else {
            Spark.publish(test_uuid, spark_status, 60, PRIVATE);
            test_last_progress_update = now;
        }
    }
}

int process_one_BCODE_command(int cmd, int index) {
    int buf, i, param1, param2, start_index, timeout;
    uint8_t integration_time, gain;

    if (cancel_process) {
        return index;
    }

    switch(cmd) {
        case 0: // Start test(integration time+gain, LED power)
            test_record.start_time = Time.now();
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);

            test_record.integration_time =  param1 & 0x00FF;
            test_record.gain =  param1>>8;
            init_sensor_with_params(&tcsAssay, pinAssaySDA, pinAssaySCL, test_record.integration_time, test_record.gain);
            init_sensor_with_params(&tcsControl, pinControlSDA, pinControlSCL, test_record.integration_time, test_record.gain);
            test_sensor_reading_count = 0;

            update_progress("Warming up sensor LEDs", 1000);
            analogWrite(pinSensorLED, param2);
            delay(1000);
            read_sensors(0); // read initial baseline values
            analogWrite(pinSensorLED, 0);
            break;
        case 1: // Delay(milliseconds)
            index = get_BCODE_token(index, &param1);
            update_progress("", param1);
            delay(param1);
            break;
        case 2: // Move(number of steps, step delay)
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);
            update_progress("Moving magnets", abs(param1) * param2 / 1000);
            move_steps(param1, param2);
            break;
        case 3: // Solenoid on(milliseconds)
            index = get_BCODE_token(index, &param1);
            update_progress("Rastering magnets", param1);
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
            update_progress("Blinking device LED", param1);
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
        case 9: // Read sensors
            read_sensors(1);
            break;
        case 10: // Read QR code
            digitalWrite(pinQRPower, HIGH);
            delay(1000);
            Serial1.begin(115200);
            digitalWrite(pinQRTrigger, LOW);
            delay(50);
            digitalWrite(pinQRTrigger, HIGH);
            timeout = Time.now() + QR_READ_TIMEOUT;
            while (!Serial1.available()) {
                if (Time.now() > timeout) {
                    return -1;
                }
                Spark.process();
            }
            i = 0;
            do {
                buf = Serial1.read();
                if (buf == -1 || test_uuid[i++] != (char) buf);
                    return -1;
                }
            } while (i < UUID_LENGTH);
            break;
        case 11: // Beep (milliseconds)
            Serial.println("Beep not implemented");
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
        case 14: // Enable sensor (integration time, gain)
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);

            test_record.integration_time =  param1;
            test_record.gain =  param2;
            init_sensor_with_params(&tcsAssay, pinAssaySDA, pinAssaySCL, test_record.integration_time, test_record.gain);
            init_sensor_with_params(&tcsControl, pinControlSDA, pinControlSCL, test_record.integration_time, test_record.gain);
            test_sensor_reading_count = 0;
            break;
        case 15: // Disable sensor
            tcsAssay.disable();
            tcsControl.disable();
            break;
        case 99: // Finish test
            test_record.finish_time = Time.now();
            write_test_record_to_flash();
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
        ERROR_MESSAGE(-15);
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
    Spark.variable("percentdone", &test_percent_complete, INT);

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
    pinMode(pinQRTrigger, OUTPUT);

    digitalWrite(pinSolenoid, LOW);
    digitalWrite(pinStepperSleep, LOW);
    digitalWrite(pinQRTrigger, HIGH);
    digitalWrite(pinQRPower, HIGH);

    Serial.begin(9600);     // standard serial port
    Serial1.begin(115200);  // QR scanner interface through RX/TX pins
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
    start_test = false;
    test_in_progress = false;
    collect_sensor_data = false;
    cancel_process = false;
    test_last_progress_update = millis();

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

    STATUS("Device initialized and ready to run test");
}

void do_run_test() {
    device_ready = false;
    start_test = false;
    test_in_progress = true;

    process_BCODE(0);

    update_progress("Test complete", -1);
    test_uuid[0] = '\0';
    test_in_progress = false;
}

void do_sensor_data_collection() {
    STATUS("Initializing sensors");
    init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
    init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
    analogWrite(pinSensorLED, brevitest.sensor_led_power);

    STATUS("Warming up sensor LEDs");
    delay(brevitest.sensor_led_warmup_ms);

    STATUS("Collecting sensor data");
    test_record.start_time = Time.now();
    test_sensor_reading_count = 0;
    CANCELLABLE(read_sensors(1);)
    test_record.finish_time = Time.now();
    CANCELLABLE(write_test_record_to_flash();)

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

    if (device_ready && start_test && !test_in_progress) {
        do_run_test();
    }

    if (collect_sensor_data) {
        do_sensor_data_collection();
    }

    if (cancel_process) {
        cancel_process = false;
    }
}
