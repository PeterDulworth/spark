#include "TCS34725.h"
#include "SoftI2CMaster.h"

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
//                         EEPROM                          //
//                                                         //
/////////////////////////////////////////////////////////////

void load_eeprom() {
  uint8_t *e = (uint8_t *) &eeprom;

  for (int addr = 0; addr < CACHE_SIZE_BYTES; addr++, e++) {
    *e = EEPROM.read(addr);
  }
}

void store_eeprom() {
  uint8_t *e = (uint8_t *) &eeprom;

  for (int addr = 0; addr < CACHE_SIZE_BYTES; addr++, e++) {
    EEPROM.write(addr, *e);
  }
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        SOLENOID                         //
//                                                         //
/////////////////////////////////////////////////////////////

void move_solenoid(int duration) {
    Spark.process();

    if (cancel_process) {
        return;
    }

    analogWrite(pinSolenoid, eeprom.param.solenoid_surge_power);
    delay(eeprom.param.solenoid_surge_period_ms);
    analogWrite(pinSolenoid, eeprom.param.solenoid_sustain_power);
    delay(duration - eeprom.param.solenoid_surge_period_ms);
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

        if (i % eeprom.param.wifi_ping_rate == 0) {
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
    delay(eeprom.param.stepper_wake_delay_ms);
}

void reset_stage() {
    STATUS("Resetting device");
    move_steps(-(long) eeprom.param.reset_steps, eeprom.param.step_delay_us);
}

void set_calibration_point() {
  uint8_t lsb, msb;

  msb = (uint8_t) (eeprom.param.calibration_steps >> 8);
  lsb = (uint8_t) (eeprom.param.calibration_steps & 0x0F);
  EEPROM.write(EEPROM_ADDR_PARAM + EEPROM_OFFSET_PARAM_CALIBRATION_STEPS, msb);
  EEPROM.write(EEPROM_ADDR_PARAM + EEPROM_OFFSET_PARAM_CALIBRATION_STEPS + 1, lsb);
}

void move_to_calibration_point() {
    CANCELLABLE(reset_stage();)
    delay(500);
    CANCELLABLE(move_steps(eeprom.param.calibration_steps, eeprom.param.step_delay_us);)
}

/////////////////////////////////////////////////////////////
//                                                         //
//                       DEVICE LED                        //
//                                                         //
/////////////////////////////////////////////////////////////

int turn_on_device_LED() {
  analogWrite(pinDeviceLED, 255);
  device_LED.currently_on = true;
  return 1;
}

int turn_off_device_LED() {
  analogWrite(pinDeviceLED, 0);
  device_LED.currently_on = false;
  return 1;
}

void start_blinking_device_LED(int total_duration) {
  device_LED.blink_timeout = Time.now() + total_duration;
  device_LED.blinking = true;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        SENSORS                          //
//                                                         //
/////////////////////////////////////////////////////////////

void init_sensor(TCS34725 *sensor, int sdaPin, int sclPin) {
    uint8_t it, gain;

    it = ((int) eeprom.param.sensor_params) & 0x00FF;
    gain = ((int) eeprom.param.sensor_params) >> 8;
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
            reading = &(eeprom.test_cache[test_index].sensor_reading_initial_assay);
        }
        else {
            reading = &(eeprom.test_cache[test_index].sensor_reading_final_assay);
        }
    }
    else {
        buffer = control_buffer;
        if (reading_code == 0) {
            reading = &(eeprom.test_cache[test_index].sensor_reading_initial_control);
        }
        else {
            reading = &(eeprom.test_cache[test_index].sensor_reading_final_control);
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

    reading->start_time = buffer[0].sample_time;
}

void read_sensors(int reading_code) { // 0 -> initial baseline, 1 -> assay
    for (int i = 0; i < SENSOR_NUMBER_OF_SAMPLES; i += 1) {
        if (cancel_process) {
            return;
        }
        read_one_sensor(&tcsAssay, 'A', i);
        read_one_sensor(&tcsControl, 'C', i);
        delay(eeprom.param.delay_between_sensor_readings_ms);
        if (reading_code == 0) {
            update_progress("Taking baseline readings", eeprom.param.delay_between_sensor_readings_ms);
        }
        else {
            update_progress("Reading test results", eeprom.param.delay_between_sensor_readings_ms);
        }
    }

    convert_samples_to_reading(reading_code, 'A');
    convert_samples_to_reading(reading_code, 'C');
}

/////////////////////////////////////////////////////////////
//                                                         //
//                  EEPROM ASSAY RECORDS                   //
//                                                         //
/////////////////////////////////////////////////////////////

int get_assay_index_by_uuid(char *uuid) {
    int i;

    for (i = 0; i < ASSAY_CACHE_SIZE; i += 1) {
        if (strncmp(uuid, eeprom.assay_cache[i].uuid, UUID_LENGTH) == 0) {
            return i;
        }
    }

    return -1;
}

void write_assay_record_to_eeprom() {
    store_assay(assay_index);
}

void store_assay(int index) {
  uint8_t *e = (uint8_t *) &eeprom.assay_cache[index];

  for (int addr = ASSAY_CACHE_INDEX; addr < (ASSAY_CACHE_INDEX + index * ASSAY_RECORD_LENGTH); addr++, e++) {
    EEPROM.write(addr, *e);
  }

  eeprom.cache_count = eeprom.cache_count & 0x0F + ((index + 1) % ASSAY_CACHE_SIZE) << 4;
  EEPROM.write(CACHE_COUNT_INDEX, eeprom.cache_count);
}

int get_assay_record_by_uuid() {
    // particle_request.param is num
    int index;

    if (particle_request.index == 0) {  // initial request
        strncpy(request_uuid, particle_request.uuid, UUID_LENGTH);
    }
    else {  // continuation request
        if (strncmp(particle_request.uuid, request_uuid, UUID_LENGTH) != 0) {
            return -3;
        }
    }

    index = get_assay_index_by_uuid(particle_request.uuid);
    if (index == -1) {
      return -2;
    }

    return process_assay_record(index);
}

int process_assay_record(int index) {
    BrevitestAssayRecord *assay;
    int len;

    assay = &eeprom.assay_cache[index];

    len = snprintf(particle_register, PARTICLE_REGISTER_SIZE, "%24s\t%64s\t%3d\t%3d\n%s\n", \
        assay->uuid, assay->name, assay->BCODE_length, assay->BCODE_version, assay->BCODE);

    particle_register[len] = '\0';
    return 0;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                  EEPROM TEST RECORDS                    //
//                                                         //
/////////////////////////////////////////////////////////////

int get_test_index_by_uuid(char *uuid) {
    int i;

    for (i = 0; i < TEST_CACHE_SIZE; i += 1) {
        if (strncmp(uuid, eeprom.test_cache[i].test_uuid, UUID_LENGTH) == 0) {
            return i;
        }
    }

    return -1;
}

void write_test_record_to_eeprom() {
    store_test(test_index);
}

void store_test(int index) {
  uint8_t *e = (uint8_t *) &eeprom.test_cache[index];

  for (int addr = TEST_CACHE_INDEX; addr < (TEST_CACHE_INDEX + index * TEST_RECORD_LENGTH); addr++, e++) {
    EEPROM.write(addr, *e);
  }

  eeprom.cache_count = eeprom.cache_count & 0xF0 + ((index + 1) % TEST_CACHE_SIZE);
  EEPROM.write(CACHE_COUNT_INDEX, eeprom.cache_count);
}

int get_test_record_by_uuid() {
    // particle_request.param is num
    int index;

    if (particle_request.index == 0) {  // initial request
        strncpy(request_uuid, particle_request.uuid, UUID_LENGTH);
    }
    else {  // continuation request
        if (strncmp(particle_request.uuid, request_uuid, UUID_LENGTH) != 0) {
            return -3;
        }
    }

    index = get_test_index_by_uuid(particle_request.uuid);
    if (index == -1) {
      return -2;
    }

    return process_test_record(index);
}

int process_test_record(int index) {
    BrevitestTestRecord *test;
    int len;

    test = &eeprom.test_cache[index];

    len = snprintf(particle_register, PARTICLE_REGISTER_SIZE, \
        "%11d\t%11d\t%24s\t%24s\t%24s\n%5d\t%5d\t%5d\t%5d\t%3d\t%3d\t%5d\t%5d\t%3d\t%3d\t%5d\n%11d\t%5d\t%5d\t%5d\n%11d\t%5d\t%5d\t%5d\n%11d\t%5d\t%5d\t%5d\n%11d\t%5d\t%5d\t%5d\n", \
        test->start_time, test->finish_time, test->test_uuid, test->cartridge_uuid, test->assay_uuid, \
//
        test->param.reset_steps, test->param.step_delay_us, test->param.wifi_ping_rate, test->param.stepper_wake_delay_ms, \
        test->param.solenoid_surge_power, test->param.solenoid_sustain_power, test->param.solenoid_surge_period_ms, \
        test->param.delay_between_sensor_readings_ms, test->param.sensor_params & 0x0F, test->param.sensor_params >> 8, test->param.calibration_steps, \
//
        test->sensor_reading_initial_assay.start_time, test->sensor_reading_initial_assay.red_norm, \
        test->sensor_reading_initial_assay.green_norm, test->sensor_reading_initial_assay.blue_norm, \
        test->sensor_reading_initial_control.start_time, test->sensor_reading_initial_control.red_norm, \
        test->sensor_reading_initial_assay.green_norm, test->sensor_reading_initial_assay.blue_norm, \
        test->sensor_reading_final_assay.start_time, test->sensor_reading_final_assay.red_norm, \
        test->sensor_reading_initial_assay.green_norm, test->sensor_reading_initial_assay.blue_norm, \
        test->sensor_reading_final_control.start_time, test->sensor_reading_final_control.red_norm, \
        test->sensor_reading_initial_assay.green_norm, test->sensor_reading_initial_assay.blue_norm);

    particle_register[len] = '\0';
    return 0;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                         PARAMS                          //
//                                                         //
/////////////////////////////////////////////////////////////

int get_param_value() {
    int index, len;
    uint16_t *value16 = &eeprom.param.reset_steps;
    uint8_t *value8 = (uint8_t *) &eeprom.param.reset_steps;

    index = extract_int_from_string(particle_command.param, 0, PARAM_CHANGE_INDEX);
    if (index < 0) {
      return -150;
    }
    if (index >= EEPROM_PARAM_LENGTH) {
      return -151;
    }

    len = extract_int_from_string(particle_request.param, PARAM_CHANGE_INDEX, PARAM_CHANGE_LENGTH);
    if (len == 1) {
        value8 += particle_request.index;
        sprintf(particle_register, "%d", *value8);
    }
    else if (len == 2) {
        if (particle_request.index % 2 == 0) {
            value16 += particle_request.index;
            sprintf(particle_register, "%d", *value16);
        }
        else {
            return -147;
        }
    }
    else {
        return -148;
    }

    return 0;
}

int get_all_param_values() {
    uint16_t *value = &eeprom.param.reset_steps;
    int i, len, index = 0;

    for (i = 0; i < EEPROM_PARAM_LENGTH; i += 2) {
        index += sprintf(&particle_register[index], "%d,", *value);
    }
    particle_register[--index] = '\0';

    return 0;
}

int change_param() {
    int index, len, value;
    uint16_t *value16 = &eeprom.param.reset_steps;
    uint8_t *value8 = (uint8_t *) &eeprom.param.reset_steps;

    index = extract_int_from_string(particle_command.param, 0, PARAM_CHANGE_INDEX);
    if (index < 0) {
      return -110;
    }
    if (index >= EEPROM_PARAM_LENGTH) {
      return -111;
    }

    len = extract_int_from_string(particle_command.param, PARAM_CHANGE_INDEX, PARAM_CHANGE_LENGTH);
    value = extract_int_from_string(particle_command.param, PARAM_CHANGE_INDEX + PARAM_CHANGE_LENGTH, PARAM_CHANGE_VALUE);
    if (len == 1) {
        value8 += index;
        *value8 = value;
    }
    else if (len == 2) {
        if (index % 2 == 0) {
            value16 += index;
            *value16 = value;
        }
        else {
            return -117;
        }
    }
    else {
        return -118;
    }

    store_params();

    return value;
}

int reset_params() {
    Param reset;

    memcpy(&eeprom.param, &reset, EEPROM_PARAM_LENGTH);
    store_params();

    return 1;
}

void load_params() {
  uint8_t *e = (uint8_t *) &eeprom.param;

  for (int addr = 0; addr < CACHE_SIZE_BYTES; addr++, e++) {
    *e = EEPROM.read(addr);
  }
}

void store_params() {
  uint8_t *e = (uint8_t *) &eeprom.param;

  for (int addr = EEPROM_ADDR_PARAM; addr < (EEPROM_ADDR_PARAM + EEPROM_PARAM_LENGTH); addr++, e++) {
    EEPROM.write(addr, *e);
  }
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        REQUESTS                         //
//                                                         //
/////////////////////////////////////////////////////////////

int get_serial_number() {
    memcpy(particle_register, eeprom.serial_number, EEPROM_SERIAL_NUMBER_LENGTH);
    particle_register[EEPROM_SERIAL_NUMBER_LENGTH] = '\0';
    return 0;
}

int parse_particle_request(String msg) {
    int len = msg.length();
    int param_length = len - REQUEST_PARAM_INDEX;

    msg.toCharArray(particle_request.arg, len + 1);

    particle_request.index = extract_int_from_string(particle_request.arg, REQUEST_INDEX_INDEX, REQUEST_INDEX_LENGTH);
    if (particle_request.index == 0) { // first request
        strncpy(particle_request.uuid, particle_request.arg, UUID_LENGTH);
        particle_request.uuid[UUID_LENGTH] = '\0';
    }
    else {
        if (strncmp(particle_request.arg, particle_request.uuid, UUID_LENGTH) != 0) {
            ERROR_MESSAGE("Register uuid mismatch");
            return -1;
        }
        if (particle_request.index == 999999) {
            particle_register[0] = '\0';
            return 0;
        }
    }

    particle_request.code = extract_int_from_string(particle_request.arg, REQUEST_CODE_INDEX, REQUEST_CODE_LENGTH);
    strncpy(particle_request.param, &particle_request.arg[REQUEST_PARAM_INDEX], param_length);
    particle_request.param[param_length] = '\0';

    return 1;
}

int request_data(String msg) {
    if (parse_particle_request(msg) > 0) {
        switch (particle_request.code) {
            case 0: // serial_number
                return get_serial_number();
            case 1: // get test record
                return get_test_record_by_uuid();
            case 2: // get assay record
                return get_assay_record_by_uuid();
            case 3: // get params
                return get_all_param_values();
            case 4: // one parameter
                return get_param_value();
        }
    }

    return -1;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                        COMMANDS                         //
//                                                         //
/////////////////////////////////////////////////////////////

int command_write_serial_number() {
    for (int i = 0; i < EEPROM_SERIAL_NUMBER_LENGTH; i += 1) {
        eeprom.serial_number[i] = particle_command.param[i];
        EEPROM.write(EEPROM_ADDR_SERIAL_NUMBER + i, particle_command.param[i]);
    }
    return 1;
}

int command_scan_QR_code() {
    // param: user_uuid
    if (memcmp(user_uuid, &particle_command.param, UUID_LENGTH) != 0) {
        return -201;
    }

    // scan QR code
    // load QR code into particle register
    // send QR code to cloud for cartridge verification
    return 0;
}

int command_claim_device() {
    // param: user_uuid, assay_uuid
    if (device_busy) {
        return -200;
    }

    device_busy = true;
    memcpy(claimant_uuid, &particle_command.param, UUID_LENGTH);
    assay_index = get_assay_index_by_uuid(&particle_command.param[UUID_LENGTH]);

    return assay_index;
}

int command_release_device() {
    claimant_uuid[0] = '\0';
    assay_index = -1;
    device_busy = false;
    return 1;
}

int command_run_brevitest() {
    if (device_busy) {
        ERROR_MESSAGE(-11);
        return -1;
    }

    start_test = true;
    update_progress("Starting test", 0);

    return 1;
}

int command_get_firmware_version() {
    int version = EEPROM.read(0);
    return version;
}

int command_cancel_brevitest() {
    cancel_process = true;
    return 1;
}

int load_buffer_from_cloud() {
    STATUS("Receiving string into buffer");

    if (btBuf.packet_number != extract_int_from_string(particle_command.param, BUFFER_PACKET_NUMBER_INDEX, BUFFER_PACKET_NUMBER_LENGTH)) {
        ERROR_MESSAGE("Buffer packet received out of order");
        ERROR_MESSAGE(particle_command.param);
        return -300;
    }
    if (btBuf.packet_number == 0) { // first packet, payload is number of packets (not including this packet) and total payload length
        btBuf.index = 0;
        memcpy(btBuf.id, &particle_command.param[BUFFER_ID_INDEX], BUFFER_ID_LENGTH);
        btBuf.number_of_packets = extract_int_from_string(particle_command.param, BUFFER_NUMBER_OF_PACKETS_INDEX, BUFFER_NUMBER_OF_PACKETS_LENGTH);
        btBuf.message_size = extract_int_from_string(particle_command.param, BUFFER_MESSAGE_SIZE_INDEX, BUFFER_MESSAGE_SIZE_LENGTH);
        memcpy(buffer_id, &particle_command.param[BUFFER_ID_INDEX], BUFFER_ID_LENGTH);
    }
    else { // payload packet, contains payload count, packet length, packet id, and payload
        if (btBuf.packet_number <= btBuf.number_of_packets) {
            if (memcmp(btBuf.id, &particle_command.param[BUFFER_ID_INDEX], BUFFER_ID_LENGTH) != 0) {
                return -301;
            }
            btBuf.payload_size = extract_int_from_string(particle_command.param, BUFFER_PAYLOAD_SIZE_INDEX, BUFFER_PAYLOAD_SIZE_LENGTH);
            memcpy(&btBuf.buffer[btBuf.index], &particle_command.param[BUFFER_PAYLOAD_INDEX], btBuf.payload_size);
            btBuf.index += btBuf.payload_size;
        }
        else {
            return -302;
        }
    }

    if (btBuf.packet_number < btBuf.number_of_packets) {
        btBuf.packet_number++;
    }
    else { // last packet
        if (btBuf.index != btBuf.message_size); {
            ERROR_MESSAGE("Buffer payload not transferred properly");
            return -303;
        }
        btBuf.packet_number = -1;
    }

    return btBuf.packet_number;
}

int command_load_assay() {
    int result;

    btBuf.packet_number = 0;
    do {
        result = load_buffer_from_cloud();
        if (result < -1) { // error generated
            return result;
        }
    } while (result != -1);

    assay_index = (eeprom.cache_count >> 4) % ASSAY_CACHE_SIZE);
    assay_index = assay_index < 0 ? 0 : assay_index;

    // populate assay struct from buffer;
}

int command_set_and_move_to_calibration_point() {
    eeprom.param.calibration_steps = extract_int_from_string(particle_command.param, 0, PARTICLE_COMMAND_PARAM_LENGTH);

    set_calibration_point();
    move_to_calibration_point();

    return 1;
}

int command_move_stage() {
  int steps = extract_int_from_string(particle_command.param, 0, PARTICLE_COMMAND_PARAM_LENGTH);
  move_steps(steps, eeprom.param.step_delay_us);
  return 1;
}

int command_energize_solenoid() {
  int duration = extract_int_from_string(particle_command.param, 0, PARTICLE_COMMAND_PARAM_LENGTH);
  move_solenoid(duration);
  return 1;
}

int command_blink_device_LED() {
  int duration = extract_int_from_string(particle_command.param, 0, PARTICLE_COMMAND_PARAM_LENGTH);
  start_blinking_device_LED(duration);
  return 1;
}

int command_read_QR_code() {
  return 1;
}

//
//

void parse_particle_command(String msg) {
    int len = msg.length();
    msg.toCharArray(particle_command.arg, len + 1);

    particle_command.code = extract_int_from_string(particle_command.arg, 0, PARTICLE_COMMAND_CODE_LENGTH);
    len -= PARTICLE_COMMAND_CODE_LENGTH;
    strncpy(particle_command.param, &particle_command.arg[PARTICLE_COMMAND_CODE_LENGTH], len);
    particle_command.param[len] = '\0';
}

int run_command(String msg) {
    parse_particle_command(msg);

    switch (particle_command.code) {
    // operating functions
        case 1: // run test
            return command_run_brevitest();
        case 2: // cancel test
            return command_cancel_brevitest();
        case 3: // claim device
            return command_claim_device();
        case 4: // release device
            return command_release_device();

    // data transfer
        case 10: // transfer data to buffer
            return command_receive_packet();
            
    // configuration functions
        case 50: // set device serial number
            return command_write_serial_number();
        case 51: // change device parameter
            return command_change_param();
        case 52: // reset device parameters to default
            return command_reset_params();
        case 53: // get current firmware version number
            return command_get_firmware_version();
        case 54: // set and move to calibration point
            return command_set_and_move_to_calibration_point();

      // test functions
        case 100:
            return command_move_stage();
        case 101:
            return command_energize_solenoid();
        case 102:
            return turn_on_device_LED();
        case 103:
            return turn_off_device_LED();
        case 104:
            return command_blink_device_LED();
        case 105:
            return command_read_QR_code();
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
    char *bcode = eeprom.assay_cache[assay_index].BCODE;

    if (cancel_process) {
        return index;
    }

    if (bcode[index] == '\0') { // end of string
        return index; // return end of string location
    }

    if (bcode[index] == '\n') { // command has no parameter
        return index + 1; // skip past parameter
    }

    // there is a parameter to extract
    i = index;
    while (i < BCODE_CAPACITY) {
        if (bcode[i] == '\0') {
            *token = extract_int_from_string(bcode, index, (i - index));
            return i; // return end of string location
        }
        if ((bcode[i] == '\n') || (bcode[i] == ',')) {
            *token = extract_int_from_string(bcode, index, (i - index));
            i++; // skip past parameter
            return i;
        }

        i++;
    }

    return i;
}

void update_progress(char *message, int duration) {
    unsigned long now = millis();
    int test_duration = eeprom.assay_cache[assay_index].duration;
    char *test_uuid = eeprom.test_cache[test_index].test_uuid;

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
        Serial.println(particle_status);
        if (now - test_last_progress_update < 1000) {
            return;
        }
        else {
            Spark.publish(test_uuid, particle_status, 60, PRIVATE);
            test_last_progress_update = now;
        }
    }
}

int process_one_BCODE_command(int cmd, int index) {
    int buf, i, param1, param2, start_index, timeout;
    BrevitestTestRecord *test_record = &eeprom.test_cache[test_index];
    uint8_t integration_time, gain;

    if (cancel_process) {
        return index;
    }

    switch(cmd) {
        case 0: // Start test(integration time+gain, LED power)
            test_record->start_time = Time.now();
            index = get_BCODE_token(index, &param1);
            index = get_BCODE_token(index, &param2);

            test_record->param.sensor_params = param1;
            integration_time =  param1 & 0x00FF;
            gain =  param1 >> 8;
            init_sensor_with_params(&tcsAssay, pinAssaySDA, pinAssaySCL, integration_time, gain);
            init_sensor_with_params(&tcsControl, pinControlSDA, pinControlSCL, integration_time, gain);

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
            move_solenoid(param1);
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
            };
            i = 0;
            do {
                buf = Serial1.read();
                if (buf == -1 || test_record->cartridge_uuid[i++] != (char) buf) {
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

            integration_time =  param1;
            gain =  param2;
            test_record->param.sensor_params = (gain << 8) + integration_time;
            init_sensor_with_params(&tcsAssay, pinAssaySDA, pinAssaySCL, integration_time, gain);
            init_sensor_with_params(&tcsControl, pinControlSDA, pinControlSCL, integration_time, gain);
            break;
        case 15: // Disable sensor
            tcsAssay.disable();
            tcsControl.disable();
            break;
        case 99: // Finish test
            test_record->finish_time = Time.now();
            write_test_record_to_eeprom();
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
  Spark.function("runcommand", run_command);
  Spark.function("requestdata", request_data);
  Spark.variable("register", particle_register, STRING);
  Spark.variable("status", particle_status, STRING);
  Spark.variable("claimant", claimant_uuid, STRING);
  Spark.variable("percentdone", &test_percent_complete, INT);

  pinMode(pinLimitSwitch, INPUT_PULLUP);
  pinMode(pinSolenoid, OUTPUT);
  pinMode(pinQRPower, OUTPUT);
  pinMode(pinSensorLED, OUTPUT);
  pinMode(pinDeviceLED, OUTPUT);
  pinMode(pinStepperSleep, OUTPUT);
  pinMode(pinStepperDir, OUTPUT);
  pinMode(pinStepperStep, OUTPUT);
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

  load_eeprom();

  device_busy = false;
  start_test = false;
  test_in_progress = false;
  cancel_process = false;
  test_last_progress_update = millis();

  particle_register[0] = '\0';

  test_index = -1;
  assay_index = -1;

  STATUS("Setup complete");
}

/////////////////////////////////////////////////////////////
//                                                         //
//                           LOOP                          //
//                                                         //
/////////////////////////////////////////////////////////////

void do_run_test() {
    start_test = false;
    test_in_progress = true;

    analogWrite(pinSolenoid, 0);
    analogWrite(pinSensorLED, 0);

    test_index = eeprom.cache_count & 0x0F;

    scan_QR_code();
    if (verify_cartridge()) {
      move_to_calibration_point();
      process_BCODE(0);

      update_progress("Test complete", -1);

      test_index = -1;
      test_in_progress = false;

      release_device();
    }
}

void update_blinking_device_LED() {
  unsigned long now = Time.now();

  if (now > device_LED.change_time) { // change LED state and reset blink timer
    if (device_LED.currently_on) {
      turn_off_device_LED();
    }
    else {
      turn_on_device_LED();
    }
    device_LED.change_time = now + DEVICE_LED_BLINK_DELAY;
  }
  if (now > device_LED.blink_timeout) {
    device_LED.blinking = false;
  }
}

void loop(){
    if (start_test && !test_in_progress) {
        do_run_test();
    }

    if (cancel_process) {
        cancel_process = false;
    }

    if (device_LED.blinking) {
      update_blinking_device_LED();
    }
}
