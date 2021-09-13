#include <Arduino_LSM9DS1.h>

#include <SPI.h>

#define SPI_CLK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
int t0, t1, dt;
int n_readings;

// SPI data sen function
void send_sample(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float mx, float my, float mz,
                  int dt, String separator)
{
  char msg_byte;
  String msg_string = "";

  msg_string.concat(ax); msg_string.concat(separator);
  msg_string.concat(ay); msg_string.concat(separator);
  msg_string.concat(az); msg_string.concat(separator);
  msg_string.concat(gx); msg_string.concat(separator);
  msg_string.concat(gy); msg_string.concat(separator);
  msg_string.concat(gz); msg_string.concat(separator);
  msg_string.concat(mx); msg_string.concat(separator);
  msg_string.concat(my); msg_string.concat(separator);
  msg_string.concat(mz); msg_string.concat(separator);
  msg_string.concat(dt); msg_string.concat('\n');
  
  // activate slave device
  digitalWrite(SS, LOW);

  // sen all msg bytes
  for (int k = 0; k < msg_string.length(); k++){
    msg_byte = msg_string.charAt(k);
    SPI.transfer(msg_byte);
  }

  // deactivate slave device
  digitalWrite(SS, HIGH);
}


void setup() {

  // initialize IMU object
  if (!IMU.begin()) {
    while (1);
  }

  digitalWrite(SS, HIGH);  // initialize SPI bus with slave deactivated
  SPI.begin(); // initialize SPI bus as master

  t0 = millis(); // initialize time counter
}


void loop() {

  n_readings = 0;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    // read IMU sensors
    n_readings += IMU.readAcceleration(ax, ay, az);
    n_readings += IMU.readGyroscope(gx, gy, gz);
    n_readings += IMU.readMagneticField(mx, my, mz);
    // check for fails in sensor reading
    if (n_readings == 3) {
      // compute measurement period dt
      t1 = millis();
      dt = t1 - t0;
      t0 = t1;
      // send all data throught SPI
      send_sample(ax, ay, az, gx, gy, gz, mx, my, mz, dt, ", ");
    }
  }

}
