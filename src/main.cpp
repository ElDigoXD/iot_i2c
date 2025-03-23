#include <Arduino.h>

#define DEBUG 1

#if DEBUG
#define LOG_VOID(val) Serial.printf(" %-40s   ------\n", #val); val
#define LOG_ACK_(val) Serial.printf(" %-40s -> %s\n", #val, val ? "NACK" : "ACK")
#define LOG_BYTE(val, buff) buff = val; Serial.printf(" %-40s -> 0x%02x\n", #val, buff)
#else // if DEBUG
#define LOG_VOID(val) val
#define LOG_ACK_(val) val
#define LOG_BYTE(val, buff) buff = val
#endif // DEBUG

void nop() {
    delayMicroseconds(5);
}

bool clock_stretching() {
    for (int retry = 0; retry < 25; retry++) {
        if (digitalRead(SCL)) {
            return true;
        }
        nop();
    }
    return false;
}

void write_bit(bool value) {
    pinMode(SDA, value ? INPUT : OUTPUT);
    nop();
    pinMode(SCL, INPUT);
    nop();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in write \n");
    }
    pinMode(SCL, OUTPUT);
}

bool read_bit() {
    pinMode(SDA, INPUT);
    nop();
    pinMode(SCL, INPUT);
    nop();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in read \n");
    }
    bool value = digitalRead(SDA);
    pinMode(SCL, OUTPUT);
    return value;
}

void i2c_start() {
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
    if (digitalRead(SDA) == 0) {
        Serial.printf("Error, SDA is low. Cannot i2c_start()\n");
    }
    pinMode(SDA, OUTPUT);
    nop();
    pinMode(SCL, OUTPUT);
}

void i2c_stop() {
    nop();
    pinMode(SDA, OUTPUT);
    pinMode(SCL, INPUT);
    nop();
    if (!clock_stretching()) {
        Serial.printf("Error, clock stretching failed in stop \n");
    }
    pinMode(SDA, INPUT);
}

uint8_t i2c_read_byte(bool ack) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data = (data << 1) | read_bit();
    }
    write_bit(!ack);
    return data;
}


bool i2c_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        write_bit(data & 0x80);
        data <<= 1;
    }
    return read_bit();
}

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

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint32_t compensate_temperature(uint32_t raw_temp) {
    const int32_t var1 =(((raw_temp >> 3) - ((int32_t)dig_T1 << 1)) * ((int32_t)dig_T2)) >> 11;
    const int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    const int32_t t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

// Adafruit_I2CDevice dev = Adafruit_I2CDevice(0x76, &Wire);




void setup() {
    //i2cInit(0, SDA, SCL, 100000);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    digitalWrite(SCL, 0);
    digitalWrite(SDA, 0);
    Serial.begin(115200);
    uint8_t data = 0;

    // Check Chip ID
    Serial.printf("Checking chip ID...\n");
    const auto chip_id = get_chip_id(ADDR);
    if (chip_id != 0x60) {
        Serial.printf("[ ERROR ] Chip ID is not 0x60 (BME280)\n");
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
    Serial.printf("    dig_T1: %u\n", dig_T1);
    LOG_BYTE(i2c_read_byte(true), data);
    dig_T1 |= data << 8;
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