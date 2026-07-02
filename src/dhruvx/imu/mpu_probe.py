from smbus2 import SMBus
import time

BUS = 13
ADDR = 0x68

with SMBus(BUS) as bus:
    for attempt in range(8):
        try:
            time.sleep(0.02)
            who = bus.read_byte_data(ADDR, 0x75)
            print("WHO_AM_I:", hex(who))
            bus.write_byte_data(ADDR, 0x6B, 0x00)
            print("Wrote PWR_MGMT_1 OK")
            break
        except Exception as e:
            print("Attempt", attempt+1, "failed:", e)
            time.sleep(0.1)
    else:
        print("All attempts failed.")
