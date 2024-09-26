import time
import os
from datetime import datetime
import pytz
import RPi.GPIO as GPIO

# Time zone for Spain
timezone = pytz.timezone("Europe/Madrid")

# Set up GPIO for fan control
GPIO.setmode(GPIO.BCM)
FAN_PIN = 23
GPIO.setup(FAN_PIN, GPIO.OUT)

# Set up PWM for fan control
pwm_frequency = 25  # 25Hz for fans
fan_pwm = GPIO.PWM(FAN_PIN, pwm_frequency)
fan_pwm.start(10)  # Start fan at 10% duty cycle

# Path for log files
log_folder = os.path.join(os.getcwd(), "logs")
if not os.path.exists(log_folder):
    os.makedirs(log_folder)


def get_cpu_temperature():
    # Read the CPU temperature from the system.
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        cpu_temp_raw = f.readline()
    return float(cpu_temp_raw) / 1000.0  # Convert to degrees Celsius


def calculate_fan_speed(cpu_temp):
    # Calculate fan speed based on temperature using linear interpolation.
    if cpu_temp <= 20.0:
        return 10  # 10% for temperatures 30  C or below
    elif cpu_temp >= 45.0:
        return 100  # 100% for temperatures 50  C or above
    else:
        # Linear interpolation between 30  C (10%) and 50  C (100%)
        return 10 + (cpu_temp - 30) * (90 / 20)  # Gradual increase between 10% and 100%


def log_temperature(cpu_temp, duty_cycle):
    # Log CPU temperature and fan duty cycle to a daily log file.
    # Get the current date and time in Spanish time
    now = datetime.now(timezone)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M:%S")

    # Log file name based on current date
    log_file_path = os.path.join(log_folder, f"{date_str}.log")

    # Create or append to the log file
    with open(log_file_path, "a") as log_file:
        log_entry = f"[{time_str}] Temp: {cpu_temp:.1f}°C, Fan: {round(duty_cycle)}%\n"
        log_file.write(log_entry)


def print_to_console(cpu_temp, duty_cycle):
    """Print CPU temperature and fan duty cycle to PM2 console."""
    now = datetime.now(timezone).strftime("%H:%M:%S")
    console_entry = f"Temp: {cpu_temp:.1f}°C, Fan: {round(duty_cycle)}%"
    print(console_entry)


try:
    last_log_time = time.time()  # Keep track of when to log to the file

    while True:
        # Read the CPU temperature
        cpu_temp = get_cpu_temperature()

        # Calculate fan speed based on CPU temperature
        current_duty_cycle = calculate_fan_speed(cpu_temp)
        fan_pwm.ChangeDutyCycle(current_duty_cycle)

        # Print temperature and fan speed to console every second
        print_to_console(cpu_temp, current_duty_cycle)

        # Log the temperature and fan speed to the file every 60 seconds
        if time.time() - last_log_time >= 60:
            log_temperature(cpu_temp, current_duty_cycle)
            last_log_time = time.time()

        # Sleep for 1 second before the next reading
        time.sleep(1)

except KeyboardInterrupt:
    pass  # Gracefully handle Ctrl+C

finally:
    # Cleanup fan PWM and GPIO
    fan_pwm.stop()
    GPIO.cleanup()
