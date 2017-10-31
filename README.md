# Overview

This library provides an interface to communicate with the Thalmic Myo,
providing the ability to scan for and connect to a nearby Myo, and giving access
to data from the EMG sensors and the IMU. For Myo firmware v1.0 or higher,
access to the output of Thalmic's own gesture recognition is also available.

The code is primarily developed on Linux.


# Installation

To install the library simply clone the repository and pip install it:

	git clone https://github.com/qtux/myo-raw.git
	cd myo-raw
	pip install .

To run the examples you will also need to install

	pip install ".[emg, classification]"


# Usage

The `myo_raw` folder contains the library files to access EMG/IMU data. The
Myo communication protocol is implemented in the MyoRaw class.

## Dongle device name

To use the library, you might need to know the name of the device
corresponding to the Myo dongle. The programs will attempt to detect it
automatically, but if that doesn't work, here's how to find it out manually:

- Linux: Run the command `ls /dev/ttyACM*`. One of the names it prints (there
  will probably only be one) is the device. Try them each if there are multiple,
  or unplug the dongle and see which one disappears if you run the command
  again. If you get a permissions error, running `sudo usermod -aG dialout
  $USER` will probably fix it.

- Windows: Open Device Manager (run `devmgmt.msc`) and look under "Ports (COM &
  LPT)". Find a device whose name includes "Bluegiga". The name you need is in
  parentheses at the end of the line (it will be "COM" followed by a number).

- Mac: Same as Linux, replacing `ttyACM` with `tty.usb`.

## Process data using handlers

To process the data, you can call **MyoRaw.add_emg_handler** or
**MyoRaw.add_imu_handler**; see `examples/emg.py` for example reference.

If your Myo has firmware v1.0 or higher, it also performs Thalmic's gesture
classification onboard, and returns that information. Use
**MyoRaw.add_arm_handler** and **MyoRaw.add_pose_handler**. Note that you
will need to perform the sync gesture after starting the program (the Myo will
vibrate as normal when it is synced).

### Perform the sync gesture as described by [Myo Support](https://support.getmyo.com/hc/en-us/articles/200755509-How-to-perform-the-sync-gesture):

> Make sure you're wearing Myo with the USB port facing your wrist. Gently flex
> your wrist away from your body. Myo will begin to vibrate when it recognizes
> this gesture. Hold this gesture for a few seconds until Myo stops vibrating.

> You will know you performed the sync gesture successfully when the Thalmic
> Labs logo LED on the armband stops pulsing. If it needs to warm up, you will
> see it blink along with an notification next to the gesture indicator window
> in Myo Connect. Once Myo is fully warmed up and synced, you will feel three
> distinct vibrations.


# Examples

Before running the examples make sure you have the `extras` requirements
installed as described above.

To run an example change directory to the `examples` folder and execute
it with python, e.g. `python emg.py`.

## emg.py (try out communication and display EMG readings)

This example provides a graphical
display of EMG readings as they come in. A command-line argument is interpreted
as the device name for the dongle; no argument means to auto-detect. You can
also press 1, 2, or 3 on the keyboard to make the Myo perform a short, medium,
or long vibration.

## classification.py (example pose classification, training program and pose event handlers)

This example contains a very basic pose classifier that uses the EMG
readings. You have to train it yourself: Make up your own poses and assign
numbers (0-9) to them. As long as a number key is pressed, the current EMG
readings will be recorded as belonging to the pose of that number. Any time a
new reading comes in, the program compares it against the stored values to
determine which pose it resembles the most. The screen displays the number of
samples currently labeled as belonging to each pose, and a histogram displaying
the classifications of the last 25 inputs. The most common classification among
the last 25 is shown in green and should be taken as the program's best estimate
of the current pose.

After you have done some training the Myo class in this file can
be used to notify a program each time a pose starts. If run as a standalone
script, it will simply print out the pose number each time a new pose is
detected. Use **Myo.add_raw_pose_handler** (rather than add_pose_handler) to be
notified of poses from this class's classifier, rather than Thalmic's onboard
processing.

### Tips for classification:

- make sure to only press the number keys while the pose is being held, not
  while your hand is moving to or from the pose
- try moving your hand around a little in the pose while recording data to give
  the program a more flexible idea of what the pose is
- the rest pose needs to be trained as a pose in itself

This method works fine as long as the Myo is not moved, but it may
take quite a large amount of training data to handle different positions well
enough.


# Issues

- on Windows, the readings become more and more delayed as time goes on
- doesn't have access to Thalmic's pose recognition (for firmware < v1.0)
- may or may not work with a Myo that has never been plugged in and set up with
  Myo Connect
- classify_myo.py segfaults on exit under certain circumstances (probably
  related to Pygame version)


# Acknowledgements

Thanks to Jeff Rowberg's example bglib implementations
(https://github.com/jrowberg/bglib/), which helped to get started with
understanding the protocol.


# License

This project is licensed under the MIT License.
