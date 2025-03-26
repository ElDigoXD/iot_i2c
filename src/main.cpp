#include <Arduino.h>

#define DEBUG 1
// Level 1: Driver, Level 2: ESP, Level 3: Adafruit
#define ABSTRACTION_LEVEL 3


#if DEBUG
#define LOG_VOID(val) Serial.printf(" %-40s   ------\n", #val); val
#define LOG_ACK_(val) Serial.printf(" %-40s -> %s\n", #val, val ? "NACK" : "ACK")
#define LOG_BYTE(val, buff) buff = val; Serial.printf(" %-40s -> 0x%02x\n", #val, buff)
#define LOG(val) val; Serial.printf(" %-40s -> %02x\n", #val, buff)

#else // if DEBUG
#define LOG_VOID(val) val
#define LOG_ACK_(val) val
#define LOG_BYTE(val, buff) buff = val
#define LOG(val) val
#endif // DEBUG


#define READ 0x01
#define WRITE 0x00
#define ADDR 0x76
#define RESET 0xB6
#define REG_ID 0xD0
#define REG_RESET 0xE0
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_TEMP 0xFA
#define REG_CALIB_00 0x88

#define MODE_NORMAL 0b11
#define MODE_SLEEP 0b00


uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint32_t compensate_temperature(uint32_t raw_temp) {
    const int32_t var1 = (((raw_temp >> 3) - ((int32_t) dig_T1 << 1)) * ((int32_t) dig_T2)) >> 11;
    const int32_t var2 = (((((raw_temp >> 4) - ((int32_t) dig_T1)) * ((raw_temp >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3)) >> 14;
    const int32_t t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

#if ABSTRACTION_LEVEL == 1

void wait() {
    delayMicroseconds(5);
}

bool clock_stretching() {
    for (int retry = 0; retry < 25; retry++) {
        if (digitalRead(SCL)) {
            return true;
        }
        wait();
    }
    return false;
}

void i2c_write_bit(bool value) {
    digitalWrite(SDA, value);
    wait();
    digitalWrite(SCL, 1);
    wait();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in write \n");
    }
    digitalWrite(SCL, 0);
    wait();
}

bool i2c_read_bit() {
    digitalWrite(SDA, 1);
    wait();
    digitalWrite(SCL, 1);
    wait();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in read \n");
    }
    bool value = digitalRead(SDA);
    digitalWrite(SCL, 0);
    return value;
}

void i2c_start() {
    digitalWrite(SDA, 1);
    wait();
    digitalWrite(SCL, 1);
    wait();
    if (digitalRead(SDA) == 0) {
        Serial.printf("Error, SDA is low. Cannot i2c_start()\n");
    }
    digitalWrite(SDA, 0);
    wait();
    digitalWrite(SCL, 0);
}

void i2c_stop() {
    wait();
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 1);
    wait();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in stop \n");
    }
    digitalWrite(SDA, 1);
}

uint8_t i2c_read_byte(bool ack) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data = (data << 1) | i2c_read_bit();
    }
    i2c_write_bit(!ack);
    return data;
}


bool i2c_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        i2c_write_bit(data & 0x80);
        data <<= 1;
    }
    return i2c_read_bit();
}



uint8_t write_then_read(const uint8_t addr, const uint8_t data) {
    uint8_t response = 0;
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((addr << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(data));

    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((addr << 1) | READ));
    LOG_BYTE(i2c_read_byte(false), response);
    LOG_VOID(i2c_stop());

    return response;
}

uint8_t get_chip_id(const uint8_t addr) { return write_then_read(addr, REG_ID); }

void reset(const uint8_t addr) {
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((addr << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(REG_RESET));
    LOG_ACK_(i2c_write_byte(RESET));
    LOG_VOID(i2c_stop());
}

void setup() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    Serial.begin(115200);

    pinMode(SCL, OUTPUT_OPEN_DRAIN);
    pinMode(SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);
    uint8_t data = 0;

    // Check Chip ID
    Serial.printf("Checking chip ID...\n");
    const auto chip_id = get_chip_id(ADDR);
    if (chip_id != 0x60) {
        Serial.printf("[ ERROR ] Chip ID is not 0x60\n");
        return;
    }
    Serial.printf("Chip ID: 0x%02x (BME280)\n", chip_id);

    // Reset the sensor
    Serial.printf("Resetting the sensor...\n");
    reset(ADDR);
    delay(100);

    //// Configure the sensor
    // Set sleep mode
    Serial.printf("Setting sleep mode...\n");
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(REG_CTRL_MEAS));
    LOG_ACK_(i2c_write_byte(MODE_SLEEP));
    LOG_VOID(i2c_stop());

    // Set temperature config
    Serial.printf("Setting temperature config...\n");
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(REG_CTRL_MEAS));
    LOG_ACK_(i2c_write_byte(0b001 << 5 | MODE_NORMAL)); // 0b001 = 1x oversampling
    LOG_VOID(i2c_stop());

    // Read calibration data
    Serial.printf("Reading calibration data...\n");
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(REG_CALIB_00));
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | READ));
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T1 = 0;
    dig_T1 = data;
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T1 |= data << 8;
    Serial.printf("    dig_T1: %u\n", dig_T1);
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T2 = 0;
    dig_T2 = data;
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T2 |= data << 8;
    Serial.printf("    dig_T2: %d\n", dig_T2);
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T3 = 0;
    dig_T3 = data;
    LOG_BYTE(i2c_read_byte(false), data);
    dig_T3 |= data << 8;
    Serial.printf("    dig_T3: %d\n", dig_T3);
    LOG_VOID(i2c_stop());
}


void loop() {
    uint32_t temp = 0;
    uint8_t data = 0;

    // Read temperature
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | WRITE));
    LOG_ACK_(i2c_write_byte(REG_TEMP));
    LOG_VOID(i2c_start());
    LOG_ACK_(i2c_write_byte((ADDR << 1) | READ));
    LOG_BYTE(i2c_read_byte(true), data);
    temp |= data;
    temp <<= 8;
    LOG_BYTE(i2c_read_byte(true), data);
    temp |= data;
    temp <<= 4;
    LOG_BYTE(i2c_read_byte(false), data);
    temp |= data >> 4;
    LOG_VOID(i2c_stop());
    temp = compensate_temperature(temp);
    Serial.printf("Compensated temperature: %.2f\n", temp/100.f);
    Serial.flush();
    // uint8_t buff = 0xD0;
    // dev.write_then_read(&buff, 1, &buff, 1);

    // printf("Adafruit_I2CDevice::read() -> %2x\n", buff);

    //uint8_t buff = 0xD0;
    //size_t err;
    //i2cWrite(0, 0x76, &buff, 1, 10);
    //i2cRead(0, 0x76, &buff, 1, 5, &err);
    //Serial.printf("i2cRead() -> %2x\n", buff);
    esp_sleep_enable_timer_wakeup(1000 * 1000);
    esp_light_sleep_start();
}

#elif ABSTRACTION_LEVEL == 2

void setup() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup(1000 * 1000);

    Serial.begin(115200);

    // esp_err_t i2cInit(uint8_t i2c_num, int8_t sda, int8_t scl, uint32_t clk_speed);
    // esp_err_t i2cWrite(uint8_t i2c_num, uint16_t address, const uint8_t* buff, size_t size, uint32_t timeOutMillis);
    // esp_err_t i2cRead(uint8_t i2c_num, uint16_t address, uint8_t* buff, size_t size, uint32_t timeOutMillis, size_t *readCount);

    i2cInit(0, SDA, SCL, 100000);

    // Check Chip ID
    uint8_t buff = REG_ID;
    size_t err;

    Serial.printf("Checking chip ID...\n");
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    if (buff != 0x60) {
        Serial.printf("[ ERROR ] Chip ID is not 0x60\n");
        return;
    }
    Serial.printf("Chip ID: 0x%02x (BME280)\n", buff);

    // Reset the sensor
    Serial.printf("Resetting the sensor...\n");
    buff = REG_RESET;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    buff = RESET;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    delay(100);

    // Set sleep mode
    Serial.printf("Setting sleep mode...\n");
    buff = REG_CTRL_MEAS;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    buff = MODE_SLEEP;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));

    // Set temperature config
    Serial.printf("Setting temperature config...\n");
    buff = REG_CTRL_MEAS;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    buff = 0b001 << 5 | MODE_NORMAL; // 0b001 = 1x oversampling
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));

    // Read calibration data
    Serial.printf("Reading calibration data...\n");
    buff = REG_CALIB_00;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T1 = buff;
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T1 |= buff << 8;
    Serial.printf("    dig_T1: %u\n", dig_T1);
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T2 = buff;
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T2 |= buff << 8;
    Serial.printf("    dig_T2: %d\n", dig_T2);
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T3 = buff;
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    dig_T3 |= buff << 8;
    Serial.printf("    dig_T3: %d\n", dig_T3);
}

void loop() {
    // Read temperature
    uint8_t buff = REG_TEMP;
    size_t err;
    LOG(i2cWrite(0, ADDR, &buff, 1, 10));
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    uint32_t temp = buff;
    temp <<= 8;
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    temp |= buff;
    temp <<= 4;
    LOG(i2cRead(0, ADDR, &buff, 1, 10, &err));
    temp |= buff >> 4;
    temp = compensate_temperature(temp);
    Serial.printf("Compensated temperature: %.2f\n", temp / 100.f);
    Serial.flush();

    esp_light_sleep_start();
}

#elif ABSTRACTION_LEVEL == 3
#undef READ
#undef WRITE
#undef ADDR
#undef RESET
#undef REG_ID
#undef REG_RESET
#undef REG_CONFIG
#undef REG_CTRL_MEAS
#undef REG_TEMP
#undef REG_CALIB_00

#undef MODE_NORMAL
#undef MODE_SLEEP

#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup(1000 * 1000);
    Serial.begin(115200);


    Wire.begin(SDA, SCL);
    bme.begin(0x76, &Wire);
    Serial.printf("Setting temperature config...\n");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_NONE,
                    Adafruit_BME280::SAMPLING_NONE,
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_0_5);
}

void loop() {
    const auto temp = bme.readTemperature();
    Serial.printf("Compensated temperature: %.2f\n", temp);
    Serial.flush();

    esp_light_sleep_start();
}
#endif // ABSTRACTION_LEVEL
