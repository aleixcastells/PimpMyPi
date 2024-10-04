import time
import os
from datetime import datetime
import pytz
import RPi.GPIO as GPIO
import numpy as np
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Time zone for Spain
timezone = pytz.timezone("Europe/Madrid")

# Set pins
FAN_PIN = 17
LED_1_PIN = 23  # LED 1 connected to GPIO pin 23
LED_2_PIN = 24  # LED 2 connected to GPIO pin 24
BTN_1_PIN = 27  # Button 1 connected to GPIO pin 27
BTN_2_PIN = 22  # Button 2 connected to GPIO pin 22

# Set up GPIO for fan control
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)

# Set up GPIO for LEDs
GPIO.setup(LED_1_PIN, GPIO.OUT)
GPIO.setup(LED_2_PIN, GPIO.OUT)

# Set up GPIO for buttons with pull-down resistors
GPIO.setup(BTN_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BTN_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize variables for LED blinking
led1_state = False
led2_state = False
led1_last_toggle_time = 0
led2_last_toggle_time = 0
led_blink_interval = 0.5  # Blink interval in seconds

# Initialize variables for button states
BTN_1 = False
BTN_2 = False

# Set up PWM for fan control
pwm_frequency = 25  # 25Hz for fans
fan_pwm = GPIO.PWM(FAN_PIN, pwm_frequency)
fan_pwm.start(10)  # Start fan at 10% duty cycle

# Path for log files
log_folder = os.path.join(os.getcwd(), "logs")
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# Initialize I2C bus and ADC
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1  # Gain setting for the ADC (+/-4.096V)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)


# Read the CPU temperature from the system.
def get_cpu_temperature():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        cpu_temp_raw = f.readline()
    return float(cpu_temp_raw) / 1000.0  # Convert to degrees Celsius


# Read the battery voltage
def read_battery_voltage():
    R1 = 9900.0  # Ohms (10K)
    R2 = 3333.0  # Ohms (3.3K)
    voltage_divider_ratio = (R1 + R2) / R2

    Vout = chan.voltage  # Voltage at the ADC input
    Vin = Vout * voltage_divider_ratio

    return Vin


# Temperature thresholds (in degrees Celsius)
MIN_TEMP = 35.0  # Minimum temperature to start increasing fan speed
MAX_TEMP = 50.0  # Maximum temperature to reach maximum fan speed

# Duty cycle thresholds (in percentage)
MIN_DUTY_CYCLE = 20.0  # Minimum duty cycle (fan speed)
MAX_DUTY_CYCLE = 100.0  # Maximum duty cycle (fan speed)


def calculate_fan_speed(cpu_temp):
    if cpu_temp > MAX_TEMP:
        return MAX_DUTY_CYCLE

    if cpu_temp < MIN_TEMP:
        return MIN_DUTY_CYCLE

    return np.interp(cpu_temp, [MIN_TEMP, MAX_TEMP], [MIN_DUTY_CYCLE, MAX_DUTY_CYCLE])


# Log CPU temperature, fan duty cycle, and battery voltage to a daily log file.
def log_temperature(cpu_temp, duty_cycle, battery_voltage):
    now = datetime.now(timezone)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M:%S")

    # Log file name based on current date
    log_file_path = os.path.join(log_folder, f"{date_str}.log")

    # Create or append to the log file
    with open(log_file_path, "a") as log_file:
        log_entry = f"[{time_str}] Temp: {cpu_temp:.1f}°C, Fan: {round(duty_cycle)}%, Battery: {battery_voltage:.2f}V\n"
        log_file.write(log_entry)


def print_to_console(cpu_temp, duty_cycle, battery_voltage):
    console_entry = f"T[{cpu_temp:.1f}] — F[{round(duty_cycle)}%] — V[{battery_voltage:.2f}] — B1[{BTN_1}] — B2[{BTN_2}] — L1[{int(cpu_temp >= MAX_TEMP)}] — L2[{int(battery_voltage < 12)}]"
    print(console_entry)


# Function to control LEDs based on temperature and battery voltage
def control_leds(cpu_temp, battery_voltage):
    global led1_state, led2_state
    global led1_last_toggle_time, led2_last_toggle_time
    current_time = time.time()

    # LED 1 blinking (overheat)
    if cpu_temp >= MAX_TEMP:
        # Check if it's time to toggle the LED
        if current_time - led1_last_toggle_time >= led_blink_interval:
            # Toggle LED 1
            led1_state = not led1_state
            GPIO.output(LED_1_PIN, led1_state)
            led1_last_toggle_time = current_time
    else:
        # Ensure LED is off
        led1_state = False
        GPIO.output(LED_1_PIN, led1_state)

    # LED 2 blinking (low battery)
    if battery_voltage < 12.0:
        # Check if it's time to toggle the LED
        if current_time - led2_last_toggle_time >= led_blink_interval:
            # Toggle LED 2
            led2_state = not led2_state
            GPIO.output(LED_2_PIN, led2_state)
            led2_last_toggle_time = current_time
    else:
        # Ensure LED is off
        led2_state = False
        GPIO.output(LED_2_PIN, led2_state)


# Initialize a list to store temperature readings
last_temps = []
N = 3  # Number of readings to average

try:
    last_log_time = time.time()  # Keep track of when to log to the file

    while True:
        # Read the CPU temperature
        cpu_temp = get_cpu_temperature()

        # Append the current temperature to the list
        last_temps.append(cpu_temp)

        # Keep only the last N readings
        if len(last_temps) > N:
            last_temps.pop(0)

        # Calculate the average temperature
        avg_cpu_temp = sum(last_temps) / len(last_temps)

        # Calculate fan speed based on the average CPU temperature
        current_duty_cycle = calculate_fan_speed(avg_cpu_temp)
        fan_pwm.ChangeDutyCycle(current_duty_cycle)

        # Read battery voltage
        battery_voltage = read_battery_voltage()

        # Control LEDs based on temperature and battery voltage
        control_leds(avg_cpu_temp, battery_voltage)

        # Read button states
        BTN_1 = GPIO.input(BTN_1_PIN)
        BTN_2 = GPIO.input(BTN_2_PIN)

        # Print temperature, fan speed, and battery voltage to console every second
        print_to_console(avg_cpu_temp, current_duty_cycle, battery_voltage)

        # Log the temperature, fan speed, and battery voltage to the file every 60 seconds
        if time.time() - last_log_time >= 60:
            log_temperature(avg_cpu_temp, current_duty_cycle, battery_voltage)
            last_log_time = time.time()

        # Sleep for 1 second before the next reading
        time.sleep(1)

except KeyboardInterrupt:
    pass  # Gracefully handle Ctrl+C

finally:
    # Cleanup fan PWM and GPIO
    fan_pwm.stop()
    GPIO.cleanup()
