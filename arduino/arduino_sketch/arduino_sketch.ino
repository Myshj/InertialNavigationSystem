
#include "I2Cdev.h"

#define SENSOR_MPU_6050
#define USE_6_AXIS

# define Sensor MPU6050
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

# if Sensor == MPU6050
// 1. 0x68 (по-умолчанию), AD0 == low
// 2. 0x69, AD0 == high
Sensor mpu;
//Sensor mpu(0x69);
# endif


//#define YPR_OUT
#define PROCESSING_OUT

#define SERIAL_SPEED 115200



#define LED_PIN 13
bool led_state = false;

bool is_dmp_ready = false;

uint8_t mpu_int_status;

uint8_t sensor_status;

uint16_t size_of_packet;

uint16_t current_buffer_size;

uint8_t fifo_buffer[64];

Quaternion quaternion; // [w, x, y, z]

VectorInt16 raw_acceleration; // [x, y, z]

VectorInt16 local_acceleration; // [x, y, z]

VectorInt16 global_acceleration; // [x, y, z]

VectorFloat gravity; // [x, y, z]

// YawPitchRoll
float ypr[3]; // [yaw, pitch, roll]

// Такие пакеты отправляем в среду Processing
uint8_t processing_packet[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpu_interrupt = false;
void dmpDataReady() {
    mpu_interrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(SERIAL_SPEED);

    while (!Serial){};
    
    Serial.println(F("Initializing sensor."));
    mpu.initialize();

    Serial.println(F("Testing sensor"));
    Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAILURE"));

    Serial.println(F("\nPress any key to continue."));

    while (Serial.available() && Serial.read()){};
    while (!Serial.available()){};
    while (Serial.available() && Serial.read()){};

    Serial.println(F("Initializing sensor's onboard processor."));
    sensor_status = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (sensor_status == 0) {
        Serial.println(F("Enabling sensor's processor."));
        mpu.setDMPEnabled(true);

        // ВНИМАНИЕ! НЕ РАБОТАЕТ НА Intel Galileo!
        Serial.println(F("Enabling interrupts."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpu_int_status = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        is_dmp_ready = true;

        size_of_packet = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = инициализация не прошла
        // 2 = конфигурация не прошла
        // В основном, сдыхает с ошибкой 1.
        Serial.print(F("DMP Initialization code: "));
        Serial.println(sensor_status);
    }

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    if (!is_dmp_ready) return;

    while (!mpu_interrupt && current_buffer_size < size_of_packet) {
    }

    mpu_interrupt = false;
    mpu_int_status = mpu.getIntStatus();

    current_buffer_size = mpu.getFIFOCount();

    if ((mpu_int_status & 0x10) || current_buffer_size == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpu_int_status & 0x02) {
        while (current_buffer_size < size_of_packet) current_buffer_size = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifo_buffer, size_of_packet);
        
        current_buffer_size -= size_of_packet;

        #ifdef YPR_OUT
            mpu.dmpGetQuaternion(&quaternion, fifo_buffer);
            mpu.dmpGetGravity(&gravity, &quaternion);
            mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef PROCESSING_OUT
            // display quaternion values in InvenSense Teapot demo format:
            processing_packet[2] = fifo_buffer[0];
            processing_packet[3] = fifo_buffer[1];
            processing_packet[4] = fifo_buffer[4];
            processing_packet[5] = fifo_buffer[5];
            processing_packet[6] = fifo_buffer[8];
            processing_packet[7] = fifo_buffer[9];
            processing_packet[8] = fifo_buffer[12];
            processing_packet[9] = fifo_buffer[13];
            Serial.write(processing_packet, 14);
            processing_packet[11]++; // packetCount, loops at 0xFF on purpose
        #endif
        
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
    }
}
