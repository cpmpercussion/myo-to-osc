"""Myo-to-OSC application.
Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000
"""
from myo import *
import datetime
import math
import logging
import argparse
import sys
from pythonosc import osc_message_builder
from pythonosc import udp_client
import serial


parser = argparse.ArgumentParser(description='Connects to a Myo, then sends EMG and IMU data as OSC messages to localhost:3000.')
parser.add_argument('-l', '--log', dest='logging', action="store_true", help='Save Myo data to a log file.')
parser.add_argument('-d', '--discover', dest='discover', action='store_true', help='Search for available Myos and print their names and MAC addresses.')
parser.add_argument('-a', '--address', dest='address', help='A Myo MAC address to connect to, in format "XX:XX:XX:XX:XX:XX".')

args = parser.parse_args()

LOG_FILE = datetime.datetime.now().isoformat().replace(":", "-")[:19] + "-myo-to-osc.log"  # Log file name.
LOG_FORMAT = '%(message)s'

osc_client = udp_client.SimpleUDPClient("localhost", 3000)  # OSC Client for sending messages.


def return_numbered_imu_processor(number=0):
    """Return a customised IMU processor sending to numbered addresses."""
    def proc_imu_custom(quat, acc, gyro):
        osc_client.send_message("/ori"+str(number), quat)
        osc_client.send_message("/acc"+str(number), acc)
        osc_client.send_message("/gyr"+str(number), gyro)
        roll, pitch, yaw = toEulerAngle(quat[0], quat[1], quat[2], quat[3])
        osc_client.send_message("/euler"+str(number), (roll / math.pi, pitch / math.pi, yaw / math.pi))  # vals sent in [-1,1] (not [-pi,pi])
        osc_client.send_message("/accmag"+str(number), vector_3d_magnitude(acc[0], acc[1], acc[2]))  # magnitude of accelerometer vector
        osc_client.send_message("/gyrmag"+str(number), vector_3d_magnitude(gyro[0], gyro[1], gyro[2]))  # magnitude of gyroscope vector
        if args.logging:
            logging.info("{3}, imu, {0[0]}, {0[1]}, {0[2]}, {0[3]}, {1[0]}, {1[1]}, {1[2]}, {2[0]}, {2[1]}, {2[2]}".format(quat, acc, gyro, datetime.datetime.now().isoformat()))  # 1 + 4 + 3 + 3 = 11 args.

    return proc_imu_custom


def return_numbered_emg_processor(number=0):
    """Return a customised EMG processor sending to numbered addresses."""
    def proc_emg_custom(emg_data):
        proc_emg = tuple(map(lambda x: x / 127.0, emg_data))  # scale EMG to be in [-1, 1]
        # print("emg:", em_data, end='\r')
        osc_client.send_message("/emg"+str(number), proc_emg)
        if args.logging:
            logging.info("{1}, emg, {0[0]}, {0[1]}, {0[2]}, {0[3]}, {0[4]}, {0[5]}, {0[6]}, {0[7]}".format(emg_data, datetime.datetime.now().isoformat()))

    return proc_emg_custom


# if args.address is not None:
#     print("Attempting to connect to Myo:", args.address)
# else:
#     print("No Myo address provided.")

# if args.logging:
#     logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format=LOG_FORMAT)
#     print("Logging enabled:", LOG_FILE)


# start the bluetooth connection
#serial_connection = serial.Serial(port="/dev/tty.usbmodem11", baudrate=115200, dsrdtr=1) # just one of these

# two bluetooth connections
#adapter = BT()

adapter1 = BT(tty="/dev/tty.usbmodem11", baudrate=115200)
adapter2 = BT(tty="/dev/tty....", baudrate=115200)


m1 = Myo(adapter=adapter1)
m2 = Myo(adapter=adapter2)


# Setup Myo Connection
# m = Myo(tty="/dev/tty.usbmodem1")  # MacOS
# m = Myo(tty="/dev/ttyACM0")  # Linux
m1.add_emg_handler(return_numbered_emg_processor(number=1))
m1.add_imu_handler(return_numbered_imu_processor(number=1))

m2.add_emg_handler(return_numbered_emg_processor(number=2))
m2.add_imu_handler(return_numbered_imu_processor(number=2))

# if args.discover:  # Discovers Myos and prints addresses, then exits script.
#     results = discover_myos(m.bt)
#     if not results:
#         print("No Myos found.")
#     sys.exit()

# m.connect(address=args.address)  # connects to specific Myo unless arg.address is none.
# # Setup Myo mode, buzzes when ready.
# m.sleep_mode(Sleep_Mode.never_sleep.value)
# # EMG and IMU are enabled, classifier is disabled (thus, no sync gestures required, less annoying buzzing).
# m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
# # Buzz to show Myo is ready.
# m.vibrate(1)

m1address = "c8:2f:84:e5:88:af"
m2address = "f4:0f:df:81:1e:1b"

m1.connect(address=m1address)  # connects to specific Myo unless arg.address is none.
m1.sleep_mode(Sleep_Mode.never_sleep.value)
m1.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
m1.vibrate(1)

m2.connect(address=m2address)  # connects to specific Myo unless arg.address is none.
m2.sleep_mode(Sleep_Mode.never_sleep.value)
m2.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.disabled.value)
m2.vibrate(1)


def run_loop():
    m1.run()
    m2.run()

print("Now running...")
try:
    while True:
        run_loop()
except KeyboardInterrupt:
    pass
finally:
    m1.disconnect()
    m2.disconnect()
    print("\nDisconnected")
