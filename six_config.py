# six_config.py

"""
Physical configuration for EZ-Robot Revolution Six.

Front of robot = camera/front dome direction.

Factory servo mapping from EZ-Robot Six assembly documentation.
"""


LEGS = {

    # Right side
    "FR": {
        "name": "Front Right",
        "hip": 0,
        "knee": 1,
        "side": "right"
    },

    "MR": {
        "name": "Middle Right",
        "hip": 15,
        "knee": 16,
        "side": "right"
    },

    "RR": {
        "name": "Rear Right",
        "hip": 12,
        "knee": 13,
        "side": "right"
    },

    # Left side
    "FL": {
        "name": "Front Left",
        "hip": 3,
        "knee": 4,
        "side": "left"
    },

    "ML": {
        "name": "Middle Left",
        "hip": 6,
        "knee": 7,
        "side": "left"
    },

    "RL": {
        "name": "Rear Left",
        "hip": 9,
        "knee": 10,
        "side": "left"
    },
}


# Six's nominal/calibration center.
NEUTRAL = 90


# Start conservatively.
HIP_STRIDE = 18

# How far to lift a foot during testing.
KNEE_LIFT = 18


# Two alternating tripods.
#
# A: Front-left + Middle-right + Rear-left
# B: Front-right + Middle-left + Rear-right

TRIPOD_A = ["FL", "MR", "RL"]
TRIPOD_B = ["FR", "ML", "RR"]
