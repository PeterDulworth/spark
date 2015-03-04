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

    if (cancel_process) {
        return;
    }

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
        if (cancel_process) {
            break;
        }
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
    delay(brevitest.solenoid_start_well_ms);
    for (int i = 0; i < number_of_rasters; i += 1) {
        if (cancel_process || limitSwitchOn()) {
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
    delay(brevitest.solenoid_finish_well_ms);
}

void move_to_next_well_and_raster(int path_length, int well_size, const char *well_name, bool first_well) {
    STATUS("Moving to %s well", well_name);
    if (first_well) {
        move_steps(path_length, brevitest.step_delay_reset_us);
    }
    else {
        move_steps(brevitest.steps_for_meniscus_transition, brevitest.step_delay_meniscus_us);
        move_steps(path_length - brevitest.steps_for_meniscus_transition, brevitest.step_delay_transit_us);
    }

    STATUS("Rastering %s well", well_name);
    CANCELLABLE(raster_well(well_size);)
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

int get_flash_assay_header_address(int count) {
    return ASSAY_RECORD_HEADER_START_ADDR + count * ASSAY_RECORD_HEADER_LENGTH;
}

int get_flash_assay_data_address(int count) {
    return ASSAY_RECORD_START_ADDR + count * ASSAY_RECORD_DATA_LENGTH;
}

int get_flash_assay_record_count() {
    int count;
    flash->read(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
    return count;
}

int get_flash_assay_record_index() {
    int count;
    flash->read(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
    count %= FLASH_RECORD_CAPACITY; // circular buffer
    return count;
}

void write_sensor_readings_to_flash(int ref_time) {
    STATUS("Writing sensor data to flash memory");

    BrevitestHeader header;

    int index = get_flash_assay_record_index();
    int header_addr = get_flash_assay_header_address(index);

    // build header
    header.num = get_flash_assay_record_count();
    header.data_addr = get_flash_assay_data_address(index);
    header.data_length = ASSAY_RECORD_DATA_LENGTH;
    header.assay_start_time = ref_time;
    header.param = brevitest;

    // write header
    flash->write(&header, header_addr, ASSAY_RECORD_HEADER_LENGTH);

    // write data
    flash->write(assay_result, header.data_addr, ASSAY_RECORD_DATA_LENGTH);

    // update record count
    header.num++;
    flash->write(&header.num, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
}

void read_sensor(TCS34725 *sensor, char sensorCode, int reading_number) {
    uint16_t clear, red, green, blue;
    int index = 2 * reading_number + (sensorCode == 'A' ? 0 : 1);
    Spark.process();
    int t = Time.now();

    STATUS("Reading %s sensor (%d of %d)", (sensorCode == 'A' ? "Assay" : "Control"), reading_number + 1, ASSAY_NUMBER_OF_SAMPLES);

    sensor->getRawData(&red, &green, &blue, &clear);

    snprintf(&assay_result[index][0], ASSAY_SAMPLE_LENGTH + 1, "%c%2d%08x%5u%5u%5u%5u", sensorCode, reading_number, t, clear, red, green, blue);
}

void collect_sensor_readings(int assay_time) {
    STATUS("Collecting sensor data");
    for (int i = 0; i < ASSAY_NUMBER_OF_SAMPLES; i += 1) {
        if (cancel_process) {
            return;
        }
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
    STATUS("Retrieving device serial number");
    spark_register[SERIAL_NUMBER_LENGTH] = '\0';
    for (int i = 0; i < SERIAL_NUMBER_LENGTH; i += 1) {
        spark_register[i] = (char) EEPROM.read(EEPROM_ADDR_SERIAL_NUMBER + i);
    }
}

void get_all_sensor_data() {
    int bufSize, i;
    int index = 0;

    STATUS("Retrieving all sensor data from last assay");
    for (i = 0; i < 2 * ASSAY_NUMBER_OF_SAMPLES; i += 1) {
        if ((SPARK_REGISTER_SIZE - index) < (ASSAY_SAMPLE_LENGTH + 1)) {
            STATUS("Buffer overrun - only partial results retrieved");
            break;
        }
        memcpy(&spark_register[index], &assay_result[i][0], ASSAY_SAMPLE_LENGTH);
        index += ASSAY_SAMPLE_LENGTH;
    }
    spark_register[index] = '\0';
}

void get_one_sensor_datum() {
    STATUS("Retrieving one sensor data point from last assay");
    int index = atoi(spark_request.param);
    memcpy(spark_register, assay_result[index], ASSAY_SAMPLE_LENGTH);
    spark_register[ASSAY_SAMPLE_LENGTH] = '\0';
}

void get_archived_assay_header() {
    STATUS("Retrieving archived assay header");

    BrevitestHeader header;
    int count, header_addr, index, num;

    count = get_flash_assay_record_count();
    if (count > 0) {
        num = atoi(spark_request.param);
        if (num < count) {
            header_addr = get_flash_assay_header_address(num);
            flash->read(&header, header_addr, ASSAY_RECORD_HEADER_LENGTH);

            sprintf(spark_register,"%4d%08x%2d%08x", header.num, header.data_addr, header.data_length, header.assay_start_time);
            spark_register[22] = '\n';
            header_addr += ASSAY_RECORD_HEADER_PARAM_OFFSET;
            get_all_params(header_addr, 23);
        }
    }
}

void get_archived_assay_record() {
    STATUS("Retrieving archived assay record");

    BrevitestHeader record;
    int count, addr, i, index, num;

    count = get_flash_assay_record_count();
    if (count > 0) {
        num = atoi(spark_request.param);
        if (num < count) {
            addr = get_flash_assay_data_address(num);
            index = 0;
            for (i = 0; i < 2 * ASSAY_NUMBER_OF_SAMPLES; i += 1) {
                if ((SPARK_REGISTER_SIZE - index) < (ASSAY_SAMPLE_LENGTH + 1)) {
                    STATUS("Buffer overrun - only partial archive data retrieved");
                    break;
                }
                flash->read(&spark_register[index], addr, ASSAY_SAMPLE_LENGTH);
                addr += ASSAY_SAMPLE_LENGTH;
                index += ASSAY_SAMPLE_LENGTH;
            }
            spark_register[index] = '\0';
        }
    }
}

void get_one_param() {
    int value;

    Serial.println("get_one_param");
    int num = extract_int_from_string(spark_request.param, 0, strlen(spark_request.param));
    flash->read(&value, num, 4);
    sprintf(spark_register, "%d", value);
    Serial.println(String(spark_register));
}

void get_all_params(int offset, int index) {
    int value, len;

    for (int i = 0; i < PARAM_TOTAL_LENGTH; i += 4) {
        flash->read(&value, i + offset, 4);
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
    if (BCODE_length == 0) {
        ERROR_MESSAGE("BCODE not found. Assay not started.");
        return -1;
    }
    if (memcmp(BCODE_test_uuid, &spark_command.param, UUID_LENGTH) != 0) {
        ERROR_MESSAGE("BCODE uuid doesn't match test uuid. Assay not started.");
        return -1;
    }

    memcpy(assay_uuid, &spark_command.param, UUID_LENGTH);
    assay_uuid[UUID_LENGTH] = '\0';
    run_assay = true;
    return 1;
}

int recollect_sensor_data() {
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
    flash->write(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
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
    flash->read(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
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
    int num, uuid_cmp;

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
        memcpy(BCODE_test_uuid, &spark_command.param[BCODE_TEST_UUID_INDEX], UUID_LENGTH);
        BCODE_index = 0;
        BCODE_length = 0;
    }
    else { // payload packet, contains payload count, packet length, uuid, and payload
        if (num <= BCODE_packets) {
            uuid_cmp = memcmp(BCODE_test_uuid, &spark_command.param[BCODE_TEST_UUID_INDEX], UUID_LENGTH);
            if (uuid_cmp != 0) {
                ERROR_MESSAGE("BCODE uuid mismatch");
                BCODE_length = 0;
                return -1;
            }
            BCODE_length = extract_int_from_string(spark_command.param, BCODE_NUM_LENGTH, BCODE_LEN_LENGTH);
            if ((BCODE_index + BCODE_length) > MAX_BCODE_BUFFER_SIZE) {
                ERROR_MESSAGE("BCODE buffer overflow");
                BCODE_length = 0;
                return -1;
            }
            memcpy(&BCODE_buffer[BCODE_index], &spark_command.param[BCODE_PAYLOAD_INDEX], BCODE_length);
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
        BCODE_buffer[BCODE_length] = '\0';
        BCODE_count = 0;
        BCODE_packets = 0;
        BCODE_index = 0;

        Serial.println(String(BCODE_buffer));
    }

    return num;
}

//
//
//  EXPOSED FUNCTIONS
//
//

int request_data(String msg) {
    char new_uuid[UUID_LENGTH + 1];

    if (spark_request.pending) {
        msg.toCharArray(new_uuid, UUID_LENGTH + 1);
        if (strcmp(new_uuid, spark_request.uuid) == 0) {
            Serial.println("Request complete: ");
            spark_register[0] = '\0';
            spark_request.pending = false;
            return 1;
        }
        else {
            ERROR_MESSAGE("Register uuid mismatch");
            return -1;
        }
    }
    else {
        Serial.print("Process request: ");
        parse_spark_request(msg);
        switch (spark_request.code) {
            case 0: // serial_number
                get_serial_number();
                break;
            case 1: // all sensor data
                get_all_sensor_data();
                break;
            case 2: // one sensor data point
                get_one_sensor_datum();
                break;
            case 3: // archived sensor header
                get_archived_assay_header();
                break;
            case 4: // archived sensor data
                get_archived_assay_record();
                break;
            case 5: // all parameters
                get_all_params(0, 0);
                break;
            case 6: // one parameter
                get_one_param();
                break;
            default:
                break;
        }
        return 1;
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
    Serial.println(String(spark_request.uuid));

    spark_request.uuid[UUID_LENGTH] = '\0';
    spark_request.code = extract_int_from_string(spark_request.arg, UUID_LENGTH, REQUEST_CODE_LENGTH);
    len -= UUID_LENGTH + REQUEST_CODE_LENGTH;
    strncpy(spark_request.param, &spark_request.arg[UUID_LENGTH + REQUEST_CODE_LENGTH], len);
    spark_request.param[len] = '\0';

    spark_request.pending = true;
}

void parse_spark_command(String msg) {
    int len = msg.length();
    msg.toCharArray(spark_command.arg, len + 1);

    spark_command.code = extract_int_from_string(spark_command.arg, 0, COMMAND_CODE_LENGTH);
    len -= COMMAND_CODE_LENGTH;
    strncpy(spark_command.param, &spark_command.arg[COMMAND_CODE_LENGTH], len);
    spark_command.param[len] = '\0';
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
    for (int i = 0; i < NUMBER_OF_PARAMS; i += 1) {
        Serial.println(*ptr++);
    }
}

int extract_int_from_string(char *str, int pos, int len) {
    char buf[12];

    strncpy(buf, &str[pos], len);
    return atoi(buf);
}

void write_assay_results_to_flash() {
    return;
}

//
//
//  BCODE
//
//

int get_BCODE_param(int index, int *param) {
    int end, i, start;

    i = index;
    while (i < MAX_BCODE_BUFFER_SIZE) {
        if (BCODE_buffer[i] == '\n' || BCODE_buffer[i] == ',') {
            break;
        }
    }

    *param = extract_int_from_string(BCODE_buffer, index, (i - index - 1));
    return (i + 1);
}

int process_one_BCODE_command(int cmd, int index) {
    int param1, param2;

    index += 2; // increment past command code

    switch(cmd) {
        case 0: // Start Assay(integration time, gain)
            assay_start_time = Time.now();
            index = get_BCODE_param(index, &param1);
            Serial.println(param1);
            index = get_BCODE_param(index, &param2);
            Serial.println(param2);
            break;
        case 1: // Delay(milliseconds)
            //
            break;
        case 2: // Move(number of steps, step delay)
            //
            break;
        case 3: // Solenoid on(milliseconds)
            //
            break;
        case 4: // Device LED on
            //
            break;
        case 5: // Device LED off
            //
            break;
        case 6: // Device LED blink(milliseconds)
            //
            break;
        case 7: // Sensor LED on(power)
            //
            break;
        case 8: // Sensor LED off
            //
            break;
        case 9: // Read sensors(number of samples)
            //
            break;
        case 10: // Read QR code
            //
            break;
        case 11: // Disable sensor
            //
            break;
        case 12: // Repeat begin(number of iterations)
            //
            break;
        case 13: // Repeat end
            //
            break;
        case 14: // Status(message length, message text)
            //
            break;
        case 99: // Finish Assay
            STATUS("Storing assay results and resetting device");
            write_assay_results_to_flash();
            reset_x_stage();
            break;
    }
}

void process_BCODE() {
    int cmd, index;

    Serial.println("Processing BCODE");
    Spark.process();
    index = 0;
    cmd = extract_int_from_string(BCODE_buffer, index, BCODE_COMMAND_LENGTH);
    if (cmd != 0) {
        cancel_process = true;
        ERROR_MESSAGE("First BCODE command must be Start Assay. Test cancelled.");
        return;
    }
    else {
        index = process_one_BCODE_command(cmd, index);
    }

    while ((cmd != 99) && (index != -1) && !cancel_process) {
        Spark.process();
        cmd = extract_int_from_string(BCODE_buffer, index, BCODE_COMMAND_LENGTH);
        index = process_one_BCODE_command(cmd, index);
    };
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
    Spark.variable("assayrunning", assay_uuid, STRING);

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

    if (EEPROM.read(0) != FIRMWARE_VERSION) {  // check for current firmware version
        EEPROM.write(0, FIRMWARE_VERSION);
    }

    flash = Devices::createWearLevelErase();

    read_params();

    int count;
    flash->read(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
    if (count == -1) { // brand new core
        count = 0;
        flash->write(&count, ASSAY_NUMBER_OF_RECORDS_ADDR, 4);
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

//
//
//  LOOP
//
//

void loop(){
    if (init_device) {
        STATUS("Initializing device");

        init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
        init_sensor(&tcsControl, pinControlSDA, pinControlSCL);

        solenoid_out();
        CANCELLABLE(reset_x_stage();)
        STATUS("Device initialized and ready to run assay");
        analogWrite(pinLED, 0);

        device_ready = true;
        init_device = false;
    }

    if (device_ready && run_assay) {
        device_ready = false;
        STATUS("Running assay...");

        process_BCODE();

        STATUS("Assay complete.");
        run_assay = false;
        assay_uuid[0] = '\0';
    }

    if (collect_sensor_data) {
        STATUS("Initializing sensors");
        init_sensor(&tcsAssay, pinAssaySDA, pinAssaySCL);
        init_sensor(&tcsControl, pinControlSDA, pinControlSCL);
        analogWrite(pinLED, brevitest.led_power);

        STATUS("Warming up sensor LEDs");
        delay(brevitest.led_warmup_ms);

        int t = Time.now();
        CANCELLABLE(collect_sensor_readings(t);)

        analogWrite(pinLED, 0);
        tcsAssay.disable();
        tcsControl.disable();
        collect_sensor_data = false;
        STATUS("Sensor data collection complete");
    }

    if (cancel_process) {
        STATUS("Process cancelled");
        cancel_process = false;
    }
}
