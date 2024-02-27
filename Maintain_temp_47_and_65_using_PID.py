import math
import smbus
import time
import sys
import RPi.GPIO as GPIO

# Constants
R0 = 10000  # Resistance at nominal temperature (in Ohms)
T0 = 298.15  # Nominal temperature in Kelvin
BETA = 3740  # Beta value of the thermistor
SET_TEMPERATURE_1 = 47  # Set temperature for first 30 seconds (in Celsius)
SET_TEMPERATURE_2 = 65  # Set temperature for second 30 seconds (in Celsius)
KP, KI, KD = 21, 0.09, 0.004 # PID constants
ADC_ADDRESS = 0x76 # Address of the ADC
ADC_CHANNEL = 0  # Channel on the ADC (for single-ended mode)
ADC_MAX = 65535  # Maximum ADC value (16-bit ADC)
PWM_FREQUENCY = 100  # PWM frequency = 100 Hz

# Initialize I2C bus
try:
    bus = smbus.SMBus(1)  # Assuming Raspberry Pi 3/4
except Exception as e:
    print("Error initializing I2C bus:", e)
    sys.exit(1)

# Set up GPIO
try:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    HEATING_PIN = 12
    PELTIER_COIL1_PIN = 15
    PELTIER_COIL2_PIN = 18
    FAN_MOSFET_GATE_PIN = 17
    GPIO.setup(HEATING_PIN, GPIO.OUT)  # Heating control
    GPIO.setup(PELTIER_COIL1_PIN, GPIO.OUT)
    GPIO.setup(PELTIER_COIL2_PIN, GPIO.OUT)
    GPIO.setup(FAN_MOSFET_GATE_PIN, GPIO.OUT)
except Exception as e:
    print("Error setting up GPIO:", e)
    sys.exit(1)

# Initialize PWM for heating control
try:
    heating_pwm = GPIO.PWM(HEATING_PIN, PWM_FREQUENCY)
    heating_pwm.start(0)  # Start PWM with duty cycle 0
except Exception as e:
    print("Error initializing PWM:", e)
    sys.exit(1)

# Initialize PID variables
previous_error = 0
time_prev = time.time()
integral_term = 0

def wait_seconds(duration):
    """Wait for the specified duration in seconds."""
    try:
        time.sleep(duration)
    except Exception as e:
        print("Error while waiting:", e)

def read_adc_value():
    """Read ADC value from the specified channel."""
    try:
        adc_raw = bus.read_i2c_block_data(ADC_ADDRESS, ADC_CHANNEL)
        adc_value = (adc_raw[0] << 8) + adc_raw[1]
        #print("ADC Value",adc_value)
        return adc_value
    except Exception as e:
        print("Error reading ADC value:", e)

def adc_to_temperature(adc_value):
    """Convert ADC value to temperature (in Celsius)."""
    try:
        resistance = R0 / ((ADC_MAX / adc_value) - 1)
        inv_temp = (1 / T0) + (1 / BETA) * math.log(resistance / R0)
        temperature = 1 / inv_temp - 248.5  # Convert to Celsius 273.15 - 24.5(room temp) = 248.75
        #print("Temperature",temperature)
        return temperature
    except Exception as e:
        print("Error converting ADC value to temperature:", e)
        return None

def calculate_pid(set_temperature, temperature_read, previous_error, integral_term, time_prev):
    """Calculate PID value based on given parameters."""
    try:
        pid_error = set_temperature - temperature_read
        #print("Pid error",pid_error)
        pid_p = KP * pid_error
        #print("pid_p",pid_p)
        elapsed_time = time.time() - time_prev 
        #print("elapsed_time",elapsed_time)
        
        elapsed_time = time.time() - time_prev 
        if elapsed_time > 0:
            integral_term += KI * pid_error * elapsed_time
            integral_term = max(min(integral_term, 100), 0)  # Anti-reset windup
            #print("integral_term",integral_term)
            pid_d = KD * ((pid_error - previous_error) / elapsed_time)
            #print("pid_d",pid_d)
        else:
            pid_d = 0
        
        pid_value = pid_p + integral_term + pid_d
        
        return pid_value, pid_error, integral_term, time.time()
    
    except Exception as e:
        print("Error calculating PID value:", e)
        return None, None, None, None

try:
    while True:
        # Logic for maintaining temperature at SET_TEMPERATURE_1 for 90 seconds
        start_time = time.time()
        while time.time() - start_time < 120:
            adc_value = read_adc_value()
            if adc_value is not None:
                temperature_read = adc_to_temperature(adc_value)
                print("Set Temperature 47 °C :", temperature_read)
                if temperature_read is not None:
                    PID_value, previous_error, integral_term, time_prev = calculate_pid(SET_TEMPERATURE_1, temperature_read,
                                                                          previous_error, integral_term, time_prev)
                    if PID_value is not None:
                        PID_value = max(0, min(100, PID_value))  # Apply constraints to PID value
                        #print("PID value:", PID_value)
                        heating_pwm.ChangeDutyCycle(PID_value)
            wait_seconds(0.3)

        # Logic for maintaining temperature at SET_TEMPERATURE_2 for next 60 seconds
        start_time = time.time()
        while time.time() - start_time < 120:
            adc_value = read_adc_value()
            if adc_value is not None:
                temperature_read = adc_to_temperature(adc_value)
                print("Set Temperature 65 °C", temperature_read)
                if temperature_read is not None:
                    PID_value, previous_error, integral_term, time_prev = calculate_pid(SET_TEMPERATURE_2, temperature_read,
                                                                          previous_error, integral_term, time_prev)
                    if PID_value is not None:
                        PID_value = max(0, min(100, PID_value))  # Apply constraints to PID value
                        #print("PID value:", PID_value)
                        heating_pwm.ChangeDutyCycle(PID_value)
            wait_seconds(0.3)

        # Turn off heating after the second 30 seconds
        heating_pwm.ChangeDutyCycle(0)
        print('Heating OFF')
        # Turn On the Peltier and Fan
        GPIO.output(PELTIER_COIL1_PIN, GPIO.HIGH)
        GPIO.output(PELTIER_COIL2_PIN, GPIO.HIGH)
        GPIO.output(FAN_MOSFET_GATE_PIN, GPIO.HIGH)

except KeyboardInterrupt:
    print("Measurement stopped by the user")
    # Clean up GPIO
    GPIO.cleanup()
except Exception as e:
    print("An unexpected error occurred:", e)
    GPIO.cleanup()

