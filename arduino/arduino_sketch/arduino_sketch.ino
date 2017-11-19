// Здесь размещены функции для общения с шиной I2C
#include "I2Cdev.h"

#define SENSOR_MPU_6050
#define USE_6_AXIS
//#define USE_9_AXIS

// Подключаем те библиотеки, которые определены типом используемого датчика и необходимым количеством степеней свободы
# define Sensor MPU6050
#include "MPU6050_6Axis_MotionApps20.h"

// Чтобы работали функции из "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Можно подключать до двух датчиков такого к одной плате (см. спецификации)
# if Sensor == MPU6050
// 1. 0x68 (по-умолчанию), AD0 == low
// 2. 0x69, AD0 == high
Sensor mpu;
//Sensor mpu(0x69); // Для второго датчика
# endif
#define YPR_OUT
#define SERIAL_SPEED 115200



#define LED_PIN 13
bool led_state = false;

/*---СЕКЦИЯ СОСТОЯНИЯ СИСТЕМЫ---*/
// Получилось ли инициализировать встроенный в датчик процессор?
bool is_dmp_ready = false;

// Здесь храним байт состояния прерывания от процессора датчика.
uint8_t mpu_int_status;

// Состояния датчика. 0 => OK
uint8_t sensor_status;

// Размер пакета, приходящего от датчика (42 байта по-умолчанию).
uint16_t size_of_packet;

// Текущий размер буфера FIFO
uint16_t current_buffer_size; // count of all bytes currently in FIFO

// В этот буфер читаем данные из датчика.
uint8_t fifo_buffer[64];

/*---Секция данных от датчика---*/
// Датчик отдаёт данные в виде кватерниона (по-умолчанию).
Quaternion quaternion; // [w, x, y, z]

// Вектор гравитации.
VectorFloat gravity; // [x, y, z]

// YawPitchRoll
float ypr[3]; // [yaw, pitch, roll]

/*---Секция обнаружения прерываний---*/
volatile bool mpu_interrupt = false;
void dmpDataReady() {
    mpu_interrupt = true;
}

uint8_t iteration = 0;

/*---Секция предустановок---*/

void setup() {
    // Подключаемся к шине I2C (I2Cdev этого автоматически не сделает!)
    // Устанавливаем частоту 400КГц I2C (200КГЦ на 8-ми МГцных процессорах)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Подключаемся к USB
    // 115200 - для корректной работы Processing
    Serial.begin(SERIAL_SPEED);
    
    // Ждём готовности интерфейса.
    while (!Serial){};
    
   // Инициализируем датчик.
    Serial.println(F("Initializing sensor."));
    mpu.initialize();

    // Проверяем подключение к датчику.
    Serial.println(F("Testing sensor"));
    Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAILURE"));

    // Ждём готовности датчика.
    Serial.println(F("\nPress any key to continue."));
    // Ждём реакции пользователя.
    while (Serial.available() && Serial.read()){};
    while (!Serial.available()){};
    while (Serial.available() && Serial.read()){};

    // Инициализируем набортный процессор.
    Serial.println(F("Initializing sensor's onboard processor."));
    sensor_status = mpu.dmpInitialize();
    
    // Устанавливаем стартовые данные для калибровки (получены эмпирическим путём)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (sensor_status == 0) {
         // Включаем набортный процессор.
        Serial.println(F("Enabling sensor's processor."));
        mpu.setDMPEnabled(true);
        
        // Включаем обнаружение прерываний.
        // ВНИМАНИЕ! НЕ РАБОТАЕТ НА Intel Galileo!
        Serial.println(F("Enabling interrupts."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpu_int_status = mpu.getIntStatus();
        
        // Считаем, что набортный процессор готов
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        is_dmp_ready = true;

        // Получаем размер пакета от датчика, чтобы знать, сколько нам потом с него читать можно за раз.
        size_of_packet = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = инициализация не прошла
        // 2 = конфигурация не прошла
        // В основном, сдыхает с ошибкой 1.
        Serial.print(F("DMP Initialization code: "));
        Serial.println(sensor_status);
    }
    
     // Пусть мигает для красоты
    pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!is_dmp_ready) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpu_interrupt && current_buffer_size < size_of_packet) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpu_interrupt = false;
    mpu_int_status = mpu.getIntStatus();

    // get current FIFO count
    current_buffer_size = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu_int_status & 0x10) || current_buffer_size == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu_int_status & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (current_buffer_size < size_of_packet) current_buffer_size = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifo_buffer, size_of_packet);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        current_buffer_size -= size_of_packet;

        if (iteration == 0){

        #ifdef YPR_OUT
            mpu.dmpGetQuaternion(&quaternion, fifo_buffer);
            mpu.dmpGetGravity(&gravity, &quaternion);
            mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
            //Serial.print("P=");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print(" Y=");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print(" R=");
            //Serial.println(ypr[2] * 180/M_PI);

            Serial.println("S="+String(0)+" P="+String(ypr[1] * 180/M_PI)+" Y="+String(ypr[0] * 180/M_PI)+" R="+String(ypr[2] * 180/M_PI));
        #endif
        }

        iteration = (iteration + 1) % 2;

        // blink LED to indicate activity
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
    }
}
