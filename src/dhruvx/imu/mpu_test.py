# mpu_test.py
from smbus2 import SMBus, i2c_msg
import time

BUS = 13                # <- use bus 13
ADDRS = [0x68, 0x69]    # common MPU addresses (AD0 low or high)

WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B

def probe_and_read(busnum):
    print("Opening I2C bus", busnum)
    with SMBus(busnum) as bus:
        found = []
        # quick probe (attempt read of WHO_AM_I)
        for a in ADDRS:
            try:
                val = bus.read_byte_data(a, WHO_AM_I)  # WHO_AM_I
                print(f"Device at 0x{a:02x} responded: WHO_AM_I = 0x{val:02x}")
                found.append(a)
            except Exception as e:
                # ignore probe failure
                pass

        if not found:
            print("No MPU found at addresses 0x68 or 0x69 on bus", busnum)
            return

        # read PWR_MGMT_1 from the first found device
        addr = found[0]
        print("Using device at 0x{:02x}".format(addr))
        try:
            p = bus.read_byte_data(addr, PWR_MGMT_1)
            print("PWR_MGMT_1 = 0x{:02x}".format(p))
            # if device asleep, wake (write 0)
            if p & 0x40:
                print("Device might be asleep; writing 0 to PWR_MGMT_1 to wake.")
                bus.write_byte_data(addr, PWR_MGMT_1, 0)
                time.sleep(0.1)
                p2 = bus.read_byte_data(addr, PWR_MGMT_1)
                print("PWR_MGMT_1 after wake = 0x{:02x}".format(p2))
        except Exception as e:
            print("Error reading/writing registers:", e)

if __name__ == "__main__":
    try:
        probe_and_read(BUS)
    except PermissionError:
        print("Permission error — try running with sudo.")
    except Exception as e:
        print("Unexpected error:", e)
