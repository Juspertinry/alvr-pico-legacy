# Changelog

## v0.1.3
- Fix repeated headset doff/don and lobby toggling wedging the video decoder
- Fix streams sometimes starting locked at a reduced framerate
- Fix oversized video packets overflowing the decoder input buffer
- Fix a decoded frame being recycled while still on screen
- New: video decoder pauses in the lobby so it idles
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
- Fix dim scenes crushing to black on the Neo 2 panel
- Match head positional prediction to controllers
- New: adaptive frame pacer

## v0.1.0
- Initial release
