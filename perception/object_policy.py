# perception/object_policy.py

"""
Defines which YOLO detections matter to Six.

IMPORTANT:
- Person detection is handled separately for safety.
- Only objects in OBSTACLE_CLASSES can trigger
  normal obstacle-navigation behavior.
"""


# ======================================================
# PERSON SAFETY
# ======================================================

PERSON_LABEL = "person"

# YOLO inference floor.
# Detections below this are discarded before
# our own policies see them.
INFERENCE_CONFIDENCE_FLOOR = 0.25

# Person detection remains independent of
# navigation obstacles.
PERSON_MIN_CONFIDENCE = 0.45


# ======================================================
# NAVIGATION OBSTACLES
# ======================================================
#
# Six should ONLY navigate around these objects.
#
# YOLO may still visually detect other classes,
# but ObstacleAnalyzer will ignore them.

OBSTACLE_CLASSES = {
    "backpack",
    "book",
    "suitcase",
    "bottle",
    "chair,"

}


# ======================================================
# SMALL OBJECTS
# ======================================================
#
# Books can occupy a relatively small part of the image,
# so let the analyzer use its small-object geometry rules.

SMALL_FLOOR_CLASSES = {
    "book",
}


# ======================================================
# CONFIDENCE
# ======================================================

DEFAULT_OBSTACLE_MIN_CONFIDENCE = 0.45


# Per-class confidence thresholds.
#
# Raise one if that class gives false positives.
# Lower one slightly if Six misses real examples.

OBSTACLE_MIN_CONFIDENCE = {

    "backpack": 0.45,

    "book": 0.50,
}