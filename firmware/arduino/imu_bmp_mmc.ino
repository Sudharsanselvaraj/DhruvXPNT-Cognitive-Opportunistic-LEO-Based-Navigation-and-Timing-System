/*
  imu_bmp_mmc.ino
  Reads:
   - SparkFun ISM330DHCX (accel + gyro)
   - SparkFun MMC5983MA (magnetometer)
   - Adafruit BMP3XX (BMP390/BMP388)
  Prints CSV lines at ~50–200 Hz depending on sensors.
*/

#include <Wire.h>

// --------------------- IMU (ISM330DHCX) ---------------------
#include <SparkFun_ISM330DHCX.h>
SparkFun_ISM330DHCX myIMU;

// --------------------- MAGNETOMETER (MMC5983MA) ---------------------
#include <SparkFun_MMC5983MA.h>        // << correct header
SparkFun_MMC5983MA mag;                // << correct class name

// --------------------- BAROMETER (BMP390/BMP388) ---------------------
#include <Adafruit_BMP3XX.h>
Adafruit_BMP3XX bmp;

const float G = 9.80665f;

unsigned long last_ms = 0;
float yaw_deg = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(200);

  // --------------------- IMU INIT ---------------------
  if (myIMU.begin() != IMU_SUCCESS) {
    Serial.println("ISM330DHCX init FAILED");
  } else {
    myIMU.setAccelRange(8);
    myIMU.setGyroRange(125);
    myIMU.setAccelODR(104);
    myIMU.setGyroODR(104);
  }

  // --------------------- MAG INIT ---------------------
  if (!mag.begin()) {
    Serial.println("MMC5983MA init FAILED");
  }

  // --------------------- BMP INIT ---------------------
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 init FAILED");
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilter(BMP3_IIR_FILTER_COEFF_3);
  }

  last_ms = millis();

  // CSV HEADER
  Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,temp_C,pitch_deg,roll_deg,mag_heading_deg,yaw_deg,baro_m");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  float ax_g = NAN, ay_g = NAN, az_g = NAN;
  float gx_dps = NAN, gy_dps = NAN, gz_dps = NAN;
  float mx_uT = NAN, my_uT = NAN, mz_uT = NAN;
  float tempC = NAN, baro_m = NAN;
  float pitch_deg = NAN, roll_deg = NAN, mag_heading_deg = NAN;

  // --------------------- READ IMU ---------------------
  if (myIMU.available()) {

    myIMU.readAccel();
    myIMU.readGyro();

    accelData_t a = myIMU.getAccel();
    gyroData_t g = myIMU.getGyro();

    ax_g = a.x / G;
    ay_g = a.y / G;
    az_g = a.z / G;

    gx_dps = g.x * 180.0f / PI;
    gy_dps = g.y * 180.0f / PI;
    gz_dps = g.z * 180.0f / PI;

    pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;
    roll_deg  = atan2f(ay_g, az_g) * 180.0f / PI;

    yaw_deg += gz_dps * dt;
    if (yaw_deg > 180) yaw_deg -= 360;
    if (yaw_deg < -180) yaw_deg += 360;
  }

  // --------------------- READ MAG ---------------------
  if (mag.isConnected()) {

    mag.getMeasurement();

    mx_uT = mag.getX_uT();
    my_uT = mag.getY_uT();
    mz_uT = mag.getZ_uT();

    if (!isnan(pitch_deg) && !isnan(roll_deg)) {
      float p = pitch_deg * PI / 180.0f;
      float r = roll_deg * PI / 180.0f;
      
      float Xh = mx_uT * cos(p) + mz_uT * sin(p);
      float Yh = mx_uT * sin(r)*sin(p) + my_uT*cos(r) - mz_uT*sin(r)*cos(p);

      mag_heading_deg = atan2f(-Yh, Xh) * 180.0f / PI;
      if (mag_heading_deg < 0) mag_heading_deg += 360;
    }
  }

  // --------------------- READ BAROMETER ---------------------
  if (bmp.isConnected() && bmp.performReading()) {
    tempC = bmp.readTemperature();
    baro_m = bmp.readAltitude(101325);
  }

  // --------------------- PRINT CSV ---------------------
  Serial.print(now); Serial.print(',');
  Serial.print(ax_g); Serial.print(',');
  Serial.print(ay_g); Serial.print(',');
  Serial.print(az_g); Serial.print(',');
  Serial.print(gx_dps); Serial.print(',');
  Serial.print(gy_dps); Serial.print(',');
  Serial.print(gz_dps); Serial.print(',');
  Serial.print(mx_uT); Serial.print(',');
  Serial.print(my_uT); Serial.print(',');
  Serial.print(mz_uT); Serial.print(',');
  Serial.print(tempC); Serial.print(',');
  Serial.print(pitch_deg); Serial.print(',');
  Serial.print(roll_deg); Serial.print(',');
  Serial.print(mag_heading_deg); Serial.print(',');
  Serial.print(yaw_deg); Serial.print(',');
  Serial.println(baro_m);

  delay(10);
}
