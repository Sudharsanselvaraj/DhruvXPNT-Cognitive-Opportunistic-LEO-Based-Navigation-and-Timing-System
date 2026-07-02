import smbus
import time

bus = smbus.SMBus(13)   # IMPORTANT: use bus 13
address = 0x68          # MPU address

# MPU6050 registers
PWR_MGMT_1 = 0x6B

# Wake up MPU
bus.write_byte_data(address, PWR_MGMT_1, 0)

def read_word(reg):
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg+1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

ACCEL_XOUT = 0x3B

while True:
    try:
        ax = read_word(ACCEL_XOUT)
        print("Accel X:", ax)
        time.sleep(0.5)
    except Exception as e:
        print("Error:", e)
        break
