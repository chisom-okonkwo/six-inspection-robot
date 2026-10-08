# six_config.py

"""
Physical and gait configuration for
EZ-Robot Revolution Six.

Front of robot = camera/front dome direction.
"""


# ======================================================
# SERVO MAP
# ======================================================

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


# ======================================================
# BASE POSE
# ======================================================

NEUTRAL = 90


# Individual mechanical alignment corrections.
#
# ARC has the concept of Servo Profiles for this exact
# purpose. Start everything at zero and tune only after
# the gait itself is working.
#
# Example:
#
# "FR": {"hip": 2, "knee": -1}

SERVO_OFFSETS = {

    "FR": {
        "hip": 0,
        "knee": 0
    },

    "MR": {
        "hip": 0,
        "knee": 0
    },

    "RR": {
        "hip": 0,
        "knee": 0
    },

    "FL": {
        "hip": 0,
        "knee": 0
    },

    "ML": {
        "hip": 0,
        "knee": 0
    },

    "RL": {
        "hip": 0,
        "knee": 0
    },
}


# ======================================================
# GAIT GEOMETRY
# ======================================================

# Hip sweep distance from center.
HIP_STRIDE = 18

# Maximum knee lift.
KNEE_LIFT = 18


# Per-leg tuning.
#
# These become extremely useful later for fixing:
#
# - slight curvature
# - mechanical differences
# - unequal servo travel
#
# Do NOT use these to hide a completely incorrect
# servo direction.

LEG_STRIDE_SCALE = {
    "FR": 1.0,
    "MR": 1.0,
    "RR": 1.0,
    "FL": 1.0,
    "ML": 1.0,
    "RL": 1.0,
}


LEG_LIFT_SCALE = {
    "FR": 1.0,
    "MR": 1.0,
    "RR": 1.0,
    "FL": 1.0,
    "ML": 1.0,
    "RL": 1.0,
}


# ======================================================
# TRIPOD GROUPS
# ======================================================

TRIPOD_A = [
    "FL",
    "MR",
    "RL"
]

TRIPOD_B = [
    "FR",
    "ML",
    "RR"
]


# ======================================================
# CONTINUOUS GAIT TIMING
# ======================================================

# Duration of one complete walking cycle.
#
# Larger = slower.
# Smaller = faster.
#
# Start here before increasing speed.
GAIT_CYCLE_SECONDS = 1.20


# How often Python sends updated poses.
#
# 30 Hz gives us 36 trajectory samples per
# 1.2 second cycle.
GAIT_UPDATE_HZ = 30


# Percentage of each leg cycle spent swinging
# through the air.
#
# 0.40 means:
#
# 40% swing
# 60% stance
#
# Because the tripods are offset by half a cycle,
# this gives short periods where BOTH tripods
# are down, improving stability.
SWING_FRACTION = 0.40


# Gently increase gait amplitude when walking
# begins instead of immediately jumping to
# the full stride.
GAIT_BLEND_SECONDS = 0.45


# ======================================================
# NORMAL POSE TRANSITIONS
# ======================================================

# Used for stand/recovery transitions.
POSE_TRANSITION_SECONDS = 0.65

POSE_TRANSITION_HZ = 40