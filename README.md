# Autonomous Indoor Safety Inspection Robot

An AI-powered autonomous hexapod inspection robot built
using an EZ-Robot Revolution Six and EZ-B v4 controller.

## Current capabilities

- Direct EZ-B control without ARC
- Custom tripod gait
- Forward/backward locomotion
- Left/right turning
- Direct EZ-B camera streaming
- OpenCV video processing
- YOLO person detection
- Concurrent perception and safety architecture
- Emergency motion preemption

## Architecture

Camera → Perception → Safety Supervisor → Motion Controller → EZ-B

## Running

Observe mode:

python -m app.autonomous_app --mode observe

Patrol mode:

python -m app.autonomous_app --mode patrol