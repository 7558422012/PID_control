import math
import smbus
import time
import sys

# Constants
R0 = 10000  # Resistance at nominal temperature (in Ohms)
T0 = 298.15  # Nominal temperature in Kelvin
BETA = 3740  # Beta value of the thermistor
ADC_ADDRESS = 0x76  # Address of the ADC
ADC_CHANNEL = 0  # Channel on the ADC (for single-ended mode)
ADC_MAX = 65535  # Maximum ADC value (16-bit ADC)

# Initialize I2C bus
try:
    bus = smbus.SMBus(1)  # Assuming Raspberry Pi 3/4
except Exception as e:
    print("Error initializing I2C bus:", e)
    sys.exit(1)

def read_adc_value():
    """Read ADC value from the specified channel."""
    try:
        adc_raw = bus.read_i2c_block_data(ADC_ADDRESS, ADC_CHANNEL)
        adc_value = (adc_raw[0] << 8) + adc_raw[1]
        print("ADC Value:", adc_value)
        return adc_value
    except Exception as e:
        print("Error reading ADC value:", e)

def adc_to_temperature(adc_value):
    """Convert ADC value to temperature (in Celsius)."""
    try:
        resistance = R0 / ((ADC_MAX / adc_value) - 1)
        inv_temp = (1 / T0) + (1 / BETA) * math.log(resistance / R0)
        temperature = 1 / inv_temp - 248.5  # Convert to Celsius 273.15 - 24.5(room temp) = 248.75
        return temperature
    except Exception as e:
        print("Error converting ADC value to temperature:", e)
        return None

try:
    while True:
        adc_value = read_adc_value()
        if adc_value is not None:
            temperature_read = adc_to_temperature(adc_value)
            print("Temperature:", temperature_read)
        time.sleep(0.3)

except KeyboardInterrupt:
    print("Measurement stopped by the user")
except Exception as e:
    print("An unexpected error occurred:", e)
