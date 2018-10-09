"""
Utility Functions for myo-to-osc

Charles P. Martin, University of Oslo, 2018.

Licensed under the MIT License.
"""
import math


# TODO: Implement other feature extraction and filtering functions here.


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
