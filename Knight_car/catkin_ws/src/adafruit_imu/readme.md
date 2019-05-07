# adafruit_imu package


# to test run:
TODO

## Dependencies
* `rospy`
* `sensor_msgs`: for the Joy.msg
* `duckietown_msgs`: for the CarControl.msg

# Node: adafruit_imu
This node reads sensor data from adafruit IMU and publishes it to sensor_msgs.Imu and sensor_msgs.MagneticField.

## Parameters
* `~pub_timestep`:
    Time steps (in seconds) between publishings of CarControl msgs. Default to 0.02 (50hz)

## Publish Topics
* `~adafruit_imu`: sensor_msgs.Imu
    Imu.angular_velocity: Vector3 of angular velocity vector.
    Imu.linear_acceleration: Vector3 of linear acceleration.
* `~adafruit_mag`: sensor_msgs.MagneticField
    MagneticField.magnetic_field: Vector3 of magnetic field.

## Services
None

