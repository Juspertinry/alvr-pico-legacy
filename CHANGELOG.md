# Changelog

## v0.1.4
- New: Trade field of view for clarity with the FOV slider in video settings
- New: In-app low battery warning at 15% and 5%
- Lower the streaming CPU & GPU clock pin to cut heat and power
- Boost streamed color saturation 15%

## v0.1.3
- New: video decoder pauses in the lobby so it idles
- Fix repeated headset doff/don and lobby toggling wedging the video decoder
- Fix streams sometimes starting locked at a reduced framerate
- Fix oversized video packets overflowing the decoder input buffer
- Fix a decoded frame being recycled while still on screen
- Lower decoder power draw and heat by cutting idle decoder wakeups
- Lower frame-timing jitter with absolute-deadline pacing
- Pin the video receive thread to fast cores to cut frame-arrival jitter
- Faster response to sleep and lobby input during a network stall
- Request a fresh keyframe cleanly after the decoder is torn down
- Reuse one ImageReader across decoders instead of leaking one per teardown

## v0.1.2
- New: drop corrupted video frames and request a keyframe instead of decoding glitched frames
- New: infer thumbstick touch from stick deflection

## v0.1.1
- New: adaptive frame pacer
- Fix dim scenes crushing to black on the Neo 2 panel
- Match head positional prediction to controllers

## v0.1.0
- Initial release
