from mpu6050 import mpu6050
import time

# MPU6050 default I2C address
MPU_ADDRESS = 0x68

# IMPORTANT: Your IMU is on I2C bus 13 (Raspberry Pi 5)
sensor = mpu6050(MPU_ADDRESS, bus=13)

print("MPU6050 Initialized on I2C Bus 13, Address 0x68")
print("Reading Accelerometer and Gyroscope Data...\n")

while True:
    try:
        # Read accelerometer data
        accel = sensor.get_accel_data()
        
        # Read gyroscope data
        gyro = sensor.get_gyro_data()
        
        # Print nicely formatted data
        print("Accel => X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(
            accel['x'], accel['y'], accel['z']
        ))
        
        print("Gyro  => X: {:.2f}  Y: {:.2f}  Z: {:.2f}".format(
            gyro['x'], gyro['y'], gyro['z']
        ))
        
        print("----------------------------------------------")
        
        time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nExiting...")
        break
