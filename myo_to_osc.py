"""Myo-to-OSC application.
Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000
"""
from myo import *
import datetime
import math
import logging
import argparse
from pythonosc import osc_message_builder
from pythonosc import udp_client

parser = argparse.ArgumentParser(description='Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000.')
parser.add_argument('-l', '--log', dest='logging', action="store_true", help='Save Myo data to a log file.')
parser.add_argument('-d', '--discover', dest='discover', action='store_true', help='Search for available Myos and print their names and MAC addresses.')
parser.add_argument('-a', '--address', dest='address', help='A Myo MAC address to connect to, in format "XX:XX:XX:XX:XX:XX".')

args = parser.parse_args()

LOG_FILE = datetime.datetime.now().isoformat().replace(":", "-")[:19] + "-myo-to-osc.log"  # Log file name.
LOG_FORMAT = '%(message)s'

osc_client = udp_client.SimpleUDPClient("localhost", 3000)  # OSC Client for sending messages.


def vector_3d_magnitude(x, y, z):
    """Calculate the magnitude of a 3d vector"""
    return math.sqrt((x * x) + (y * y) + (z * z))


def toEulerAngle(w, x, y, z):
    """ Quaternion to Euler angle conversion borrowed from wikipedia.
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles """
    # roll (x-axis rotation)
    sinr = +2.0 * (w * x + y * z)
    cosr = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y-axis rotation)
    sinp = +2.0 * (w * y - z * x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny = +2.0 * (w * z + x * y)
    cosy = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def proc_imu(quat, acc, gyro):
    osc_client.send_message("/ori", quat)
    osc_client.send_message("/acc", acc)
    osc_client.send_message("/gyr", gyro)
    roll, pitch, yaw = toEulerAngle(quat[0], quat[1], quat[2], quat[3])
    osc_client.send_message("/euler", (roll / math.pi, pitch / math.pi, yaw / math.pi))  # vals sent in [-1,1] (not [-pi,pi])
    osc_client.send_message("/accmag", vector_3d_magnitude(acc[0], acc[1], acc[2]))  # magnitude of accelerometer vector
    osc_client.send_message("/gyrmag", vector_3d_magnitude(gyro[0], gyro[1], gyro[2]))  # magnitude of gyroscope vector
    if args.logging:
        logging.info("{3}, imu, {0[0]}, {0[1]}, {0[2]}, {0[3]}, {1[0]}, {1[1]}, {1[2]}, {2[0]}, {2[1]}, {2[2]}".format(quat, acc, gyro, datetime.datetime.now().isoformat()))  # 1 + 4 + 3 + 3 = 11 args.


def proc_emg(emg_data):
    proc_emg = tuple(map(lambda x: x / 127.0, emg_data))  # scale EMG to be in [-1, 1]
    # print("emg:", em_data, end='\r')
    osc_client.send_message("/emg", proc_emg)
    if args.logging:
        logging.info("{1}, emg, {0[0]}, {0[1]}, {0[2]}, {0[3]}, {0[4]}, {0[5]}, {0[6]}, {0[7]}".format(emg_data, datetime.datetime.now().isoformat()))


def proc_battery(battery_level):
    # print("Battery", battery_level, end='\r')
    osc_client.send_message("/battery", battery_level)

if args.address is not None:
    print("Attempting to connect to Myo:", args.address)
else:
    print("No Myo address provided.")

if args.logging:
    logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format=LOG_FORMAT)
    print("Logging enabled:", LOG_FILE)
# Setup Myo Connection
m = Myo()  # scan for USB bluetooth adapter and start the serial connection automatically
# m = Myo(tty="/dev/tty.usbmodem1")  # MacOS
# m = Myo(tty="/dev/ttyACM0")  # Linux
m.add_emg_handler(proc_emg)
m.add_imu_handler(proc_imu)
m.add_battery_handler(proc_battery)

if args.discover:  # Discovers Myos and prints addresses.
    discover_myos(m.bt)

m.connect(address=args.address)  # connects to specific Myo unless arg.address is none.
# Setup Myo mode, buzzes when ready.
m.sleep_mode(Sleep_Mode.never_sleep.value)
# EMG and IMU are enabled, classifier is disabled (thus, no sync gestures required, less annoying buzzing).
m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
# Buzz to show Myo is ready.
m.vibrate(1)

def run_loop():
    m.run()

print("Now running...")
try:
    while True:
        run_loop()
except KeyboardInterrupt:
    pass
finally:
    m.disconnect()
    print("\nDisconnected")


# TODO:
#   - move classification if then to myohw.py
#   - experiment connecting to multiple myos.
