# ALVR for Pico Neo 2

A fork of the ALVR client with support for the Pico Neo 2 and Neo 2 Eye.
Works with [ALVR server 20.14.1](https://github.com/alvr-org/ALVR/releases/tag/v20.14.1).

## Features

- Lobby accessible from anywhere with a double-press of the right app button
- In-app 16-band audio equalizer
- Software IPD adjustment
- Eye tracking (Neo 2 Eye)
- Diagnostic HUD (latency and system stats)


## Third-party binaries

This repository includes prebuilt native libraries under `app/src/main/jniLibs/`
that are not covered by the MIT license:

- `libPvr_UnitySDK.so` (PICO / Pico Interactive SDK)
- `libtracking_module.so` (PICO / Pico Interactive SDK)

These are property of Pico Interactive / PICO and are subject to PICO's own SDK
license terms. If you are the rights holder and object to their inclusion, open
an issue and they will be removed.
