"""
Myo Hardware Bluetooth Specification from thalmic labs.
Following: https://github.com/thalmiclabs/myo-bluetooth/
The API is Copyright (c) 2015, Thalmic Labs Inc, originally released under a. BSD 3-clause "New" or "Revised" License
Python version 2017.
"""

from enum import Enum
from struct import pack, unpack


# Characteristic handles from MyoLinux
class MyoChars(Enum):
    """Handles for Bluetooth Characteristics for the Myo. Borrowed from MyoLinux."""
    # ControlService
    MyoInfoCharacteristic = 0x00
    DeviceName = 0x03
    BatteryCharacteristic = 0x11
    BatteryDescriptor = 0x12
    FirmwareVersionCharacteristic = 0x17
    CommandCharacteristic = 0x19
    # ImuDataService
    IMUDataCharacteristic = 0x1c
    IMUDataDescriptor = 0x1d
    # Arm Notifications
    ClassifierCharacteristic = 0x23
    ArmDescriptor = 0x24
    # EMG.
    EMGCharacteristic = 0x27  # Extra Mystery EMG handle Used in handle_data.
    EMGDescriptor = 0x28  # Marked as hidden EMG notification. Subscribe to EMG notifications.
    EmgData0Characteristic = 0x2b
    EmgData1Characteristic = 0x2e
    EmgData2Characteristic = 0x31
    EmgData3Characteristic = 0x34
    EmgData0Descriptor = 0x2c
    EmgData1Descriptor = 0x2f
    EmgData2Descriptor = 0x32
    EmgData3Descriptor = 0x35


MYO_EMG_CHARACTERISTICS = [MyoChars.EmgData0Characteristic.value,
                           MyoChars.EmgData1Characteristic.value,
                           MyoChars.EmgData2Characteristic.value,
                           MyoChars.EmgData3Characteristic.value]


MYO_SERVICE_INFO_UUID = [
    0x42, 0x48, 0x12, 0x4a,
    0x7f, 0x2c, 0x48, 0x47,
    0xb9, 0xde, 0x04, 0xa9,
    0x01, 0x00, 0x06, 0xd5]

MyoServiceInfoUuid = pack('BBBBBBBBBBBBBBBB', *MYO_SERVICE_INFO_UUID)

# The number of EMG sensors that a Myo has.
num_emg_sensors = 8

# myo_hardware Myo Hardware Data Structures
# These types and enumerations describe the format of data sent to and from a Myo device using Bluetooth Low Energy.
# All values are big-endian.

# The following enum lists the 16bit short UUIDs of Myo services and characteristics. To construct a full 128bit
# UUID, replace the two 0x00 hex bytes of MYO_SERVICE_BASE_UUID with a short UUID from standard_services.
# The byte sequence of MYO_SERVICE_BASE_UUID is in network order. Keep this in mind when doing the replacement.
# For example, the full service UUID for Services.ControlService would be d5060001-a904-deb9-4748-2c7f4a124842.
MYO_SERVICE_BASE_UUID = [
    0x42, 0x48, 0x12, 0x4a,
    0x7f, 0x2c, 0x48, 0x47,
    0xb9, 0xde, 0x04, 0xa9,
    0x00, 0x00, 0x06, 0xd5]


class Services(Enum):
    ControlService                = 0x0001  # Myo info service
    MyoInfoCharacteristic         = 0x0101  # Serial number for this Myo and various parameters which
                                            # are specific to this firmware. Read-only attribute.
                                            # See fw_info_t.
    FirmwareVersionCharacteristic = 0x0201  # Current firmware version. Read-only characteristic.
                                            # See fw_version_t.
    CommandCharacteristic         = 0x0401  # Issue commands to the Myo. Write-only characteristic.
                                            # See command_t.

    ImuDataService                = 0x0002  # IMU service
    IMUDataCharacteristic         = 0x0402  # See imu_data_t. Notify-only characteristic.
    MotionEventCharacteristic     = 0x0502  # Motion event data. Indicate-only characteristic.

    ClassifierService             = 0x0003  # Classifier event service.
    ClassifierEventCharacteristic = 0x0103  # Classifier event data. Indicate-only characteristic. See pose_t.

    EmgDataService                = 0x0005  # Raw EMG data service.
    EmgData0Characteristic        = 0x0105  # Raw EMG data. Notify-only characteristic.
    EmgData1Characteristic        = 0x0205  # Raw EMG data. Notify-only characteristic.
    EmgData2Characteristic        = 0x0305  # Raw EMG data. Notify-only characteristic.
    EmgData3Characteristic        = 0x0405  # Raw EMG data. Notify-only characteristic.


class Standard_Services(Enum):
    """ Standard Bluetooth device services """
    BatteryService = 0x180f  # Battery service
    BatteryLevelCharacteristic = 0x2a19  # Current battery level information. Read/notify characteristic.
    DeviceName = 0x2a00  # Device name data. Read/write characteristic.


class Pose(Enum):
    """ Supported Poses. """
    rest = 0x0000
    fist = 0x0001
    wave_in = 0x0002
    wave_out = 0x0003
    fingers_spread = 0x0004
    double_tap = 0x0005
    unknown = 0xffff


def fw_info(data):
    """ Various parameters that may affect the behaviour of this Myo armband.
    The Myo library reads this attribute when a connection is established.
    Value layout for the att_handle_fw_info attribute. """
    serial_number, unlock_pose, active_classifier_type, active_classifier_index, has_custom_classifier, stream_indicating, sku, reserved = unpack('BBBBBBHBBBBBB', data)
    return {
        "serial_number": serial_number,  # Unique serial number of this Myo.
        "unlock_pose": unlock_pose,  # Pose that should be interpreted as the unlock pose. See pose_t.
        "active_classifier_type": active_classifier_type,  # Whether Myo is currently using a built-in or a custom classifier.
                                                           # See classifier_model_type_t.
        "active_classifier_index": active_classifier_index,  # Index of the classifier that is currently active.
        "has_custom_classifier": has_custom_classifier,  # Whether Myo contains a valid custom classifier. 1 if it does, otherwise 0.
        "stream_indicating": stream_indicating,  # Set if the Myo uses BLE indicates to stream data, for reliable capture.
        "sku": sku,  # SKU value of the device. See Sku.
        "reserved": reserved  # Reserved for future use; populated with zeros.
    }
# {
#     uint8_t serial_number[6];        ///< Unique serial number of this Myo.
#     uint16_t unlock_pose;            ///< Pose that should be interpreted as the unlock pose. See pose_t.
#     uint8_t active_classifier_type;  ///< Whether Myo is currently using a built-in or a custom classifier.
#                                      ///< See classifier_model_type_t.
#     uint8_t active_classifier_index; ///< Index of the classifier that is currently active.
#     uint8_t has_custom_classifier;   ///< Whether Myo contains a valid custom classifier. 1 if it does, otherwise 0.
#     uint8_t stream_indicating;       ///< Set if the Myo uses BLE indicates to stream data, for reliable capture.
#     uint8_t sku;                     ///< SKU value of the device. See Sku.
#     uint8_t reserved[7];             ///< Reserved for future use; populated with zeros.
# } fw_info_t;
# STATIC_ASSERT_SIZED(fw_info_t, 20);


class Sku(Enum):
    """ Known Myo SKUs. """
    unknown = 0  # Unknown SKU (default value for old firmwares)
    black_myo = 1  # Black Myo
    white_myo = 2  # White Myo


class Hardware_Rev(Enum):
    """ Known Myo hardware revisions. """
    unknown = 0  # Unknown hardware revision.
    revc = 1  # Myo Alpha (REV-C) hardware.
    revd = 2  # Myo (REV-D) hardware.
    # num_hardware_revs         # Number of hardware revisions known; not a valid hardware revision.


def fw_version(data):
    """ Version information for the Myo firmware.
    Value layout for the att_handle_fw_version attribute.
    Minor version is incremented for changes in this interface.
    Patch version is incremented for firmware changes that do not introduce changes in this interface. """
    major, minor, patch, hardware_rev = unpack('HHHH', data)
    # Myo hardware revision. See hardware_rev.
    return major, minor, patch, hardware_rev

FIRMWARE_VERSION_MAJOR = 1
FIRMWARE_VERSION_MINOR = 2


# control_commands Control Commands


class Command(Enum):
    """ Kinds of Myo Commands """
    set_mode = 0x01  # Set EMG and IMU modes. See set_mode_t.
    vibrate = 0x03  # Vibrate. See vibrate_t.
    deep_sleep = 0x04  # Put Myo into deep sleep. See deep_sleep_t.
    vibrate2 = 0x07  # Extended vibrate. See vibrate2_t.
    set_sleep_mode = 0x09  # Set sleep mode. See set_sleep_mode_t.
    unlock = 0x0a  # Unlock Myo. See unlock_t.
    user_action = 0x0b  # Notify user that an action has been recognized / confirmed. See user_action_t.


def command_header(command, payload_size):
    """ Header that every command begins with. """
    #  command - Command to send. See Command.
    #  payload_size - Number of bytes in payload.
    return pack('BB', command, payload_size)


class EMG_Mode(Enum):
    """ EMG Modes. """
    none = 0x00  # Do not send EMG data.
    send_emg = 0x02  # Send filtered EMG data.
    send_emg_raw = 0x03  # Send raw (unfiltered) EMG data.


class IMU_Mode(Enum):
    """ IMU modes. """
    none = 0x00  # Do not send IMU data or events.
    send_data = 0x01  # Send IMU data streams (accelerometer, gyroscope, and orientation).
    send_events = 0x02  # Send motion events detected by the IMU (e.g. taps).
    send_all = 0x03  # Send both IMU data streams and motion events.
    send_raw = 0x04  # Send raw IMU data streams.


class Classifier_Mode(Enum):
    """Classifier Modes. """
    disabled = 0x00  # Disable and reset the internal state of the onboard classifier.
    enabled = 0x01  # Send classifier events (poses and arm events).


def command_set_mode(emg_mode, imu_mode, classifier_mode):
    """ Command to set EMG and IMU modes. """
    header = command_header(Command.set_mode.value, 3)  # command == set_mode. payload_size = 3.
    payload = pack('BBB', emg_mode, imu_mode, classifier_mode)
    # EMG sensor mode. See Emg_Mode
    # IMU mode. See Imu_Mode.
    # Classifier mode. See classifier_mode_t.
    return header + payload


class Vibration_Type(Enum):
    """ Kinds of vibrations. """
    vib_none = 0x00  # Do not vibrate.
    vib_short = 0x01  # Vibrate for a short amount of time.
    vib_medium = 0x02  # Vibrate for a medium amount of time.
    vib_long = 0x03  # Vibrate for a long amount of time.


def command_vibrate(type):
    """ Vibration command."""
    header = command_header(Command.vibrate.value, 1)  # command == command_vibrate. payload_size == 1.
    payload = pack('B', type)  # See Vibration_Type
    return header + payload


def command_deep_sleep():
    """ Deep sleep command. """
    return command_header(Command.deep_sleep.value, 0)  # command == command_deep_sleep. payload_size == 0.


# TODO: Fix this second vibrate command.
# def command_vibrate2(duration, strength):
#     """ Extended vibration command. """
#     COMMAND_VIBRATE2_STEPS = 6
#     header = command_header(Command.vibrate2.value, 18)  # command == command_vibrate2. payload_size == 18.
#     steps = pack('HB', 
#         duration,  # duration (in ms) of the vibration
#         strength)  # strength of vibration (0 - motor off, 255 - full speed)
#     # TODO: this is wrong, the steps bit doesn't work.
#     return
# #     command_header_t header;
# #     struct PACKED {
# #         uint16_t duration;
# #         uint8_t strength;
# #     } steps[COMMAND_VIBRATE2_STEPS];
# # } command_vibrate2_t;
# # STATIC_ASSERT_SIZED(command_vibrate2_t, 20);


class Sleep_Mode(Enum):
    """ Sleep modes. """
    normal = 0x00  # Normal sleep mode; Myo will sleep after a period of inactivity.
    never_sleep = 0x01  # Never go to sleep.


def command_set_sleep_mode(sleep_mode):
    """ Set sleep mode command. """
    header = command_header(Command.set_sleep_mode.value, 1)  # command == set_sleep_mode. payload_size == 1.
    payload = pack('B', sleep_mode)  # Sleep mode. See Sleep_Mode.
    return header + payload


class Unlock_Type(Enum):
    """ Unlock types. """
    lock = 0x00  # Re-lock immediately.
    timed = 0x01  # Unlock now and re-lock after a fixed timeout.
    hold = 0x02  # Unlock now and remain unlocked until a lock command is received.


def command_unlock(unlock_type):
    """ Unlock Myo command.
    Can also be used to force Myo to re-lock. """
    header = command_header(Command.unlock.value, 1)  # command == command_unlock. payload_size == 1.
    payload = pack('B', unlock_type)  # Unlock type. See Unlock_Type.
    return header + payload


class User_Action_Type(Enum):
    """ User action types. """
    user_action_single = 0  # User did a single, discrete action, such as pausing a video.


def command_user_action(user_action_type):
    """ User action command. """
    header = command_header(Command.user_action.value, 1)  # command == command_user_action. payload_size == 1.
    payload = pack('B', user_action_type)  # Type of user action that occurred. See user_action_type_t.
    return header + payload


class Classifier_Model_Type(Enum):
    """ Classifier model types. """
    builtin = 0  # Model built into the classifier package.
    custom = 1  # Model based on personalized user data.


def imu_data(data):
    """ Unpack Integrated motion data. """
    # todo
    values = unpack('<10h', data)  # This is 20 bytes divided into 10 16bit values.
    quaternion = values[:4]  # Orientation data, represented as a unit quaternion. Values are multiplied by ORIENTATION_SCALE.
    acceleration = values[4:7]  # Accelerometer data. In units of g. Range of + -16. Values are multiplied by ACCELEROMETER_SCALE.
    gyroscope = values[7:10]  # Gyroscope data. In units of deg/s. Range of + -2000. Values are multiplied by GYROSCOPE_SCALE.
    return quaternion, acceleration, gyroscope


# Default IMU sample rate in Hz.
DEFAULT_IMU_SAMPLE_RATE = 50

# Scale values for unpacking IMU data
ORIENTATION_SCALE = 16384.0  # See imu_data_t::orientation
ACCELEROMETER_SCALE = 2048.0  # See imu_data_t::accelerometer
GYROSCOPE_SCALE = 16.0  # See imu_data_t::gyroscope


class Motion_Event_Type(Enum):
    """ Types of motion events. """
    motion_event_tap = 0x00


def motion_event(data):
    """ Motion event data received in a att_handle_motion_event attribute. """
    # For motion_event_tap events.
    # Event_specific data.
    event_type, tap_direction, tap_count = unpack('<3B', data)
    # See Motion_Event_Type for event_type
    return event_type, tap_direction, tap_count


class Classifier_Event_Type(Enum):
    """ Types of classifier events. """
    arm_synced = 0x01
    arm_unsynced = 0x02
    pose = 0x03
    unlocked = 0x04
    locked = 0x05
    sync_failed = 0x06


class Arm(Enum):
    """ Enumeration identifying a right arm or left arm. """
    right = 0x01
    left = 0x02
    unknown = 0xff


class X_Direction(Enum):
    """ Possible directions for Myo's +x axis relative to a user's arm. """
    toward_wrist = 0x01
    toward_elbow = 0x02
    unknown = 0xff


class Sync_Result(Enum):
    """ Possible outcomes when the user attempts a sync gesture. """
    sync_failed_too_hard = 0x01  # Sync gesture was performed too hard.


def classifier_event(data):
    """" Classifier event data received in a att_handle_classifier_event attribute. """
    cla_type, event_value, x_direction, _, _, _ = unpack('<6B', data)  # only first three bytes are in spec, what's with the second three?
    return cla_type, event_value, x_direction
#     uint8_t type; ///< See classifier_event_type_t
#     /// Event-specific data
#     union PACKED {
#         /// For classifier_event_arm_synced events.
#         struct PACKED {
#             uint8_t arm; ///< See arm_t
#             uint8_t x_direction; ///< See x_direction_t
#         };

#         /// For classifier_event_pose events.
#         uint16_t pose; ///< See pose_t

#         /// For classifier_event_sync_failed events.
#         uint8_t sync_result; ///< See sync_result_t.
#     };
# } classifier_event_t;
# STATIC_ASSERT_SIZED(classifier_event_t, 3);

# The rate that EMG events are streamed over Bluetooth.
EMG_DEFAULT_STREAMING_RATE = 200


def emg_data(data):
    """ Unpacks raw data received in an att_handle_emg_data attribute."""
    emg1 = unpack('<8b', data[:8])  # 1st sample of EMG data. #     int8_t sample1[8];
    emg2 = unpack('<8b', data[8:])  # 2nd sample of EMG data. #     int8_t sample2[8];
    # should these be unpacked when they arrive?
    return emg1, emg2
