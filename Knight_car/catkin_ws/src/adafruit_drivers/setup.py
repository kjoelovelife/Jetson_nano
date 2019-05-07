## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['Adafruit_ADS1x15', 'Adafruit_GPIO','Adafruit_I2C','Adafruit_LSM303','Adafruit_MotorHAT','Adafruit_PWM_Servo_Driver','Gyro_L3GD20'],
    package_dir={'': 'include'},
)

setup(**setup_args)
