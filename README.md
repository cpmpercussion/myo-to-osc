# Overview

This project provides a bridge between the Thalmic Myo Armband and OSC connected applications such as Pd, SuperCollider, or Max/MSP. The code is pure-Python designed to run in MacOS, Linux, (maybe also Windows) using the Myo bluetooth dongle.

Please note that this project *requires* Python 3, and uses the `pyserial` and `pythonosc` modules.

# Usage

Install the dependencies:

`pip3 install requirements.txt`

Start the program:

`python3 myo_to_osc.py`

It will connect to the first Myo it sees and then start sending OSC messages for the EMG and IMU sensors.

Open `rec_myo.pd` in Pure Data to see an example of reading these OSC messages.

## Dongle device name

This program requires the Myo's included USB Bluetooth LE dongle which provides a simple serial interface for connecting to the Myo.

The `BT` class should be able to find the serial port of the dongle automatically, if it doesn't try unplugging and plugging it back in or specifying the address manually. You can start the Myo class with the adapter's device name as an argument if you want to be sure.

### Pose / Arm Classification

This library can use the Myo's onboard pose and arm recognition. You have to turn this feature on using the `set_mode` function of the Myo class. e.g., 

    m.set_mode(EMG_Mode.send_emg.value, IMU_Mode.send_data.value, Classifier_Mode.enabled.value)

Then perform the sync gesture as described by [Myo Support](https://support.getmyo.com/hc/en-us/articles/200755509-How-to-perform-the-sync-gesture):

> Make sure you're wearing Myo with the USB port facing your wrist. Gently flex
> your wrist away from your body. Myo will begin to vibrate when it recognizes
> this gesture. Hold this gesture for a few seconds until Myo stops vibrating.

> You will know you performed the sync gesture successfully when the Thalmic
> Labs logo LED on the armband stops pulsing. If it needs to warm up, you will
> see it blink along with an notification next to the gesture indicator window
> in Myo Connect. Once Myo is fully warmed up and synced, you will feel three
> distinct vibrations.

# Acknowledgements

Thanks to the original authors of the `myo_raw` library, and later contributions that served as a starting point for this project: Danny Zhu, Alvaro Villoslada, Fernando Cosentino.

# License

This project is licensed under the MIT License.
