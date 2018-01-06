# Overview

This project provides a bridge between the Thalmic Myo Armband and OSC connected applications such as Pd, SuperCollider, or Max/MSP. The code is purely Python designed to run in MacOS and Linux using the Myo bluetooth dongle.

Please note that this project *requires* Python 3.

# Usage

Install the dependencies:

`pip install -r requirements.txt`

Start the program:

`python3 myo_to_osc.py`

It will connect to the first Myo it sees and then start sending OSC messages for the EMG and IMU sensors.

Open `rec_myo.pd` in Pure Data to see an example of reading these OSC messages.

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

- MacOS: Same as Linux, replacing `ttyACM` with `tty.usb`.

### Perform the sync gesture as described by [Myo Support](https://support.getmyo.com/hc/en-us/articles/200755509-How-to-perform-the-sync-gesture):

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
