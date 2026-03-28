// Include required libraries
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "i2c_handler.h"
#include "utils.h"
#include "qmc5883l.h"
#include "mpu6500.h"
#include "neo_6m.h"
#include "imu.h"
#include "navigation.h"
#include "my_locations.h"
#include "display.h"

// OLED global declarations
#define SSD1306_ADDR 0x3c // I2C address of OLED
#define SSD1306_SCREEN_WIDTH 128 // Screen width
#define SSD1306_SCREEN_HEIGHT 64 // Screen height
#define SSD1306_RESET -1 // OLED reset macro

// Create SSD1306 object
Adafruit_SSD1306 ssd1306(SSD1306_SCREEN_WIDTH, SSD1306_SCREEN_HEIGHT, &Wire, SSD1306_RESET);

// I2C global declarations
#define SDA_PIN 21 // I2C SDA pin
#define SCL_PIN 22 // I2C SCL pin
#define I2C_FREQ 100000 // I2C clock speed, standard mode (HZ)

// QMC5883L global declarations
#define QMC5883L_ADDR 0x2C // I2C address of QMC5883L
#define QMC5883L_CALIBRATION_TIME 5000 
#define QMC5883L_OUTPUT_RATE 10 // Data output in Hz
#define QMC5883L_OVERSAMPLE_RATE 8 // Oversample rate
#define QMC5883L_DOWNSAMPLE_RATE 8 // Downsample rate
#define QMC5883L_RANGE 8 // +/- range in Gauss

// Create QMC5883L object
QMC5883L qmc5883l(QMC5883L_ADDR);

// MPU6500 global declarations
#define MPU6500_ADDR 0x68 // I2C address of MPU6500
#define MPU6500_SAMPLE_RATE_DIVIDER 9 // Eq. to 100 Hz output rate
#define MPU6500_FIFO_MODE 0 // Overwrite old data
#define MPU6500_ACCEL_BANDWIDTH 44 // Accel LPF cutoff frequency
#define MPU6500_ACCEL_RANGE 2 // +/- range in m/s^2
#define MPU6500_TEMP_DISABLE 0 // Disable temp sensor
#define MPU6500_GYRO_DISABLE 0 // Disable gyro sensor

// Create MPU6500 object
MPU6500 mpu6500(MPU6500_ADDR);

// Neo6M global declarations
#define NEO6M_SERIAL_PORT Serial1
#define NEO6M_BAUD_RATE 9600
#define NEO6M_RX 16 // TODO: Confirm what pin I want to use
#define NEO6M_TX 17 // TODO: Confirm what pin I want to use

// Create Neo6M object
Neo6M neo6m(NEO6M_SERIAL_PORT, NEO6M_BAUD_RATE, NEO6M_RX, NEO6M_TX);

// Struct for storing sensor information
Vec3 accel, mag;

// Keep track of setup errors
bool setupSuccess = true;
bool qmc5883lSuccess = true;
bool mpu6500Success = true;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(1000);

  // Initialize Neo6M Serial
  neo6m.begin();

  // Initialize I2C
  if(!i2cInit(SDA_PIN, SCL_PIN, I2C_FREQ)){
    Serial.println("I2C failed to initialize!");
    setupSuccess = false;
  }

  // Initialize ssd1306
  if(!ssd1306.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDR)){
    Serial.println("SSD1306 failed to initialize!");
    setupSuccess = false;
  }

  // Scan for I2C devices (if necessary)
  // i2cScan();

  // Configure qmc5883l settings
  qmc5883lSuccess &= qmc5883l.resetRegisters();
  qmc5883lSuccess &= qmc5883l.setMode(QMC5883L::MODE_CONTINUOUS);
  qmc5883lSuccess &= qmc5883l.setOutputRate(QMC5883L_OUTPUT_RATE);
  qmc5883lSuccess &= qmc5883l.setOverSampleRate(QMC5883L_OVERSAMPLE_RATE);
  qmc5883lSuccess &= qmc5883l.setDownSampleRate(QMC5883L_DOWNSAMPLE_RATE);
  qmc5883lSuccess &= qmc5883l.setSetResetMode(QMC5883L::SET_ON);
  qmc5883lSuccess &= qmc5883l.setRange(QMC5883L_RANGE);
  setupSuccess &= qmc5883lSuccess;
  if(!qmc5883lSuccess){
    Serial.println("QMC5883L failed to initialize!");
  }

  // Configure mpu6500 settings
  mpu6500Success &= mpu6500.resetRegisters();
  mpu6500Success &= mpu6500.setSampleRateDivider(MPU6500_SAMPLE_RATE_DIVIDER);
  mpu6500Success &= mpu6500.setFIFOMode(MPU6500_FIFO_MODE);
  mpu6500Success &= mpu6500.setFSync(MPU6500::EXT_SOURCE_DISABLE);
  mpu6500Success &= mpu6500.setAccelLPF(1, MPU6500_ACCEL_BANDWIDTH);
  mpu6500Success &= mpu6500.setAccelRange(MPU6500_ACCEL_RANGE);
  mpu6500Success &= mpu6500.enableGyro(MPU6500_GYRO_DISABLE);
  mpu6500Success &= mpu6500.enableTempSense(MPU6500_TEMP_DISABLE);
  setupSuccess &= mpu6500Success;
  if(!mpu6500Success){
    Serial.println("MPU6500 failed to initialize!");
  }

  // Calibrate qmc5883l
  // Comment out if calibration isn't necessary
  if(qmc5883lSuccess){
    qmc5883l.calibrate(QMC5883L_CALIBRATION_TIME);
  }
}

void loop() {
  // Setup fail condition
  if(!setupSuccess){
    Serial.println("Setup failed!");
    while(true){}
  }

  // Get magnetometer readings
  if(qmc5883l.isDRDY() == 1){
    qmc5883l.read();
    mag.x = qmc5883l.getX();
    mag.y = qmc5883l.getY();
    mag.z = qmc5883l.getZ();
  }

  // Get accelerometer readings
  if(mpu6500.isDRDY() == 1){
    mpu6500.readAccel();
    accel.x = mpu6500.getAccelX();
    accel.y = mpu6500.getAccelY();
    accel.z = mpu6500.getAccelZ();
  }

  // Get tilt-compensated azimuth
  float azimuth = true_azimuth(accel.x, accel.y, accel.z, mag.x, mag.y, mag.z);

  Serial.println("START READING");

  // Print accelerometer readings
  Serial.print("X accel: ");
  Serial.print(accel.x);
  Serial.print(" Y accel: ");
  Serial.print(accel.y);
  Serial.print(" Z accel: ");
  Serial.println(accel.z);

  // Print magnetometer readings
  Serial.print("X mag: ");
  Serial.print(mag.x);
  Serial.print(" Y mag: ");
  Serial.print(mag.y);
  Serial.print(" Z mag: ");
  Serial.println(mag.z);

  // Print tilt-compensated azimuth
  Serial.print("Corrected azimuth: ");
  Serial.println((int)azimuth);

  Serial.println("END READING");

  // Read Neo6M
  if(neo6m.isDRDY()){
    neo6m.read();
  } else {
    Serial.println("Neo6M information not available!");
  }

  Location myLocation = {neo6m.getLatitude(), neo6m.getLongitude()};

  // Find heading and distance to target
  Location target = closestTarget(myLocation, LOCATIONS, nLocations);
  float heading = targetHeading(azimuth, myLocation, target);
  float distance = targetDistance(myLocation, target);

  // Display on OLED
  updateCompass(azimuth, heading, distance, ssd1306);

  delay(200);
}