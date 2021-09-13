#include "Wire.h" // Arduino I2C library
#include "SPI.h" // Arduino SPI library
#include <AS5600.h> // angular position sensor AS5600 library
#include <Servo.h> // servo motors library

// build the servo objecs
Servo servoX;
Servo servoY;  
Servo servoZ;

// define each servo PWM pin
#define SERVOX_PIN 3
#define SERVOY_PIN 5
#define SERVOZ_PIN 6

// sensor AS5600 working voltage (3.3 o 5)
#define SYS_VOL 3.3

// build the AS5600 sensor object
AMS_5600 ams5600;

// TCA9548A multiplexer I2C address
#define I2C_MUX_ADDR 0x70 

// TCA9548A multiplexer buses to connect each AS5600 sensor
#define AS5600X_BUS 0
#define AS5600Y_BUS 1
#define AS5600Z_BUS 2

// parameters
const float servox_freq = 1.5, servoy_freq = 2.0, servoz_freq = 1.0; // servos osillating frecuency
const float servox_orig = 90, servoy_orig = 90, servoz_orig = 90; // servos central angle
const float servox_amp = 90, servoy_amp = 45, servoz_amp = 60; // servos angle max range
const float angx_off = 187.0, angy_off = 169.0, angz_off = 0.0; // AS5600 sensors offset

// variables
int nsamples = 0; // samples counter
float t; // time input/output
float angx, angy, angz; // AS5600 angles input
float servox_ang, servoy_ang, servoz_ang; // servos angle output
float ax, ay, az, gx, gy, gz, mx, my, mz; // imu values input
float bx, by, bz; // gyro bias

// TCA9548A multiplexer bus selection function
void i2c_mux_select (uint8_t bus) {
  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << bus); // send a byte to select the bus
  Wire.endTransmission();
}

// AS5600 sensor checking function
void as5600_check (uint8_t bus) {
  Serial.print("Checking AS5600 sensor #"); Serial.print(bus); Serial.println(" ...");
  i2c_mux_select(bus); // select bus to use
  while(true){
    if(ams5600.detectMagnet() == 1 ){
        Serial.print("+-> Magnet detected. Current Magnitude: ");
        Serial.println(ams5600.getMagnitude());
        break;
    }else{
        Serial.println("+-> Can not detect magnet! Retrying...");
    }
    delay(2500);
  }
  Serial.println();
}

// AS5600 sensor reading function
float as5600_read(uint8_t bus) {
  i2c_mux_select(bus); // sselect bus to use
  word rawAngle = ams5600.getRawAngle(); // read the AS5600 sensor
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float degAngle = rawAngle * 0.087; // convert raw data to degrees
  return degAngle;
}

// read AS5600 sensors
void read_angle_sensors() {
  angx = as5600_read(AS5600X_BUS) - angx_off;
  angy = as5600_read(AS5600Y_BUS) - angy_off;
  angz = servoz_ang - 90.0;
}

// compute angle and move servos
void move_servos() {
    // compute servos angle
    servox_ang = servox_amp*sin(2*PI*servox_freq*t) + servox_orig;
    servoy_ang = servoy_amp*sin(2*PI*servoy_freq*t) + servoy_orig;
    servoz_ang = servoz_amp*sin(2*PI*servoz_freq*t) + servoz_orig;
    // write servos angle
    servoX.write(servox_ang);
    servoY.write(servoy_ang);
    servoZ.write(servoz_ang);
}

// SPI bus reading main variables
char SPI_dataReceived [100];
volatile byte SPI_index;
volatile bool SPI_receptionEnd;
String SPI_dataString;

// SPI bus interrupt routine
ISR (SPI_STC_vect) {
  // read received data from the SPDR register of the SPI bus
  byte SPI_byteReceived = SPDR; 
  // insert received data to buffer
  if (SPI_index < sizeof SPI_dataReceived)  {
    SPI_dataReceived[SPI_index++] = SPI_byteReceived;
    // if end line received, proces the buffer
    if (SPI_byteReceived == '\n') {
      SPI_receptionEnd = true;
    }
  }
}

// send all data throught serial and reset SPI bus
void send_data_serial(String separator) {
  if (SPI_receptionEnd)  {
      // store SPI data
      SPI_dataReceived[SPI_index] = 0;
      SPI_dataString = String(SPI_dataReceived); // convert to string
      // reset SPI bus variables
      SPI_index = 0;
      SPI_receptionEnd = false;
      // send data throught Serial
      Serial.print(separator);
      Serial.print(t); Serial.print(separator);
      Serial.print(angx); Serial.print(separator);
      Serial.print(angy); Serial.print(separator);
      Serial.print(angz); Serial.print(separator);
      SPI_dataString.replace(", ", separator); // replace Arduino BLE 33 separator
      Serial.print(SPI_dataString);
      nsamples ++; // increase samples counter
  }
}

void setup() {
  // initialize the serial port
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial port (UART) initilized!");

  // initialize the I2C bys
  Wire.begin();
  Serial.println("I2C bus initilized!");
  
  // activate SPI bus in slave mode
  SPCR |= bit (SPE);
  // configure the MISO pin as output (slave out)
  pinMode (MISO, OUTPUT);
  // initialize the SPI bus buffer
  SPI_index = 0;
  // initialize the reception bit
  SPI_receptionEnd = false;
  // activate the SPI bus interruption
  SPI.attachInterrupt();
  Serial.println("SPI bus initilized!");

  Serial.println();

  // attach each servo to pins
  servoX.attach(SERVOX_PIN);
  servoY.attach(SERVOY_PIN);
  servoZ.attach(SERVOZ_PIN);
  // initilize servos in the center position
  Serial.println("Moving servos to origin...");
  servoX.write(servox_orig);
  servoY.write(servoy_orig);
  servoZ.write(servoz_orig);
  // wait to start
  delay(2000);

  Serial.println();

  // check AS5600 sensors
  as5600_check(AS5600X_BUS);
  as5600_check(AS5600Y_BUS);
  //as5600_check(AS5600Z_BUS);

  Serial.println("Senging static data...");
  delay(1000);
  // send 100 samples static
  while (nsamples <= 100) {
    t = millis()/1000.0;  
    angx = 0.0; angy = 0.0; angz = 0.0; 
    send_data_serial(" ");
  }
}

void loop() {
  t = millis()/1000.0;
  move_servos();
  read_angle_sensors();
  send_data_serial(" ");
}
