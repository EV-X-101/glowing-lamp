from smbus2 import SMBus, i2c_msg
import time

I2C_ADDRESS = 0x48  # Default I2C address for ADS1115, change if necessary

def read_adc(channel):
    with SMBus(1) as bus:  # 1 indicates /dev/i2c-1
        # Write configuration register
        # 0x01: Pointer to config register
        # 0x8383: Start a single conversion
        # - 0x8000: operational status/single-shot conversion start
        # - 0x0100: input multiplexer configuration (AINp=AIN0 and AINn=AIN1)
        # - 0x0020: programmable gain amplifier configuration (+/-4.096V)
        # - 0x0003: data rate (128 SPS)
        msg = i2c_msg.write(I2C_ADDRESS, [0x01, 0x83, 0x83])
        bus.i2c_rdwr(msg)

        # Sleep for a bit to let the conversion complete
        time.sleep(0.05)

        # Read conversion register
        # 0x00: Pointer to conversion register
        msg = i2c_msg.write(I2C_ADDRESS, [0x00])
        bus.i2c_rdwr(msg)

        msg = i2c_msg.read(I2C_ADDRESS, 2)
        bus.i2c_rdwr(msg)

        # Combine the two bytes of the response
        result = msg.buf[0][0] << 8 | msg.buf[1][0]

        # The ADS1115 returns a 16-bit signed value, so if the high bit is
        # set we need to subtract 65536 to get the negative value.
        if result >= 0x8000:
            return result - 65536
        else:
            return result

while True:
    print(read_adc(0))
    time.sleep(1)
