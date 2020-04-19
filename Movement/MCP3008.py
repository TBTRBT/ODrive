import spidev

# Define Variables
delay = 0.5

# Create SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000


def readadc(adcnum):
    # read SPI data from the MCP3008, 8 channels in total
    if adcnum > 7 or adcnum < 0:
        return -1
    r = spi.xfer2([1, 8 + adcnum << 4, 0])
    data = ((r[1] & 3) << 8) + r[2]
    return data
