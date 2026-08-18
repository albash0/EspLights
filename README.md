# ESP Lights

This repository contains two firmware variants:

- `encrypted/`: AES-128-CCMP encrypted ESP-NOW unicast with fixed peers and replay/duplicate rejection.
- `unencrypted/`: the original unencrypted ESP-NOW broadcast implementation.

Each variant contains:

- `controller/`: handheld ESP32-C6 controller.
- `receiver/`: receiver A, the ESP32-C3 receiver with presence, temperature,
  humidity, and microphone support.
- `light_receiver/`: receiver L, a light-only ESP32 driving one strip divided
  into the same three logical zones.

On the controller, edit the cell directly below the zone selector to choose
`A`, `L`, or `BOTH`. The selected receiver receives all subsequent lighting
commands. Receiver L implements Music as a sensitivity-controlled pulse because
it intentionally has no microphone or other sensors. Automatic presence
control applies only to receiver A.

## Encrypted setup

Copy each `secrets.example.h` to `secrets.h`. All controllers and the receiver
share one random 16-byte PMK. Give every controller/receiver pair a unique
random 16-byte LMK. Each receiver lists its controllers in `CONTROLLER_PEERS`.
Each controller lists receiver A and receiver L in `RECEIVER_PEERS`, including
the receiver ID, station MAC, and matching LMK.

Receiver A sends telemetry separately to every configured controller. Replay
state is tracked independently per controller, so equal sequence numbers from
different controllers do not conflict. If multiple controllers send settings,
the most recent valid command wins.

The installed ESP32 Arduino core supports up to six encrypted peers per device.
Builds stop with a clear compile-time error if a configured list is larger.

The unencrypted broadcast variant already accepts commands from multiple
controllers, but it cannot authenticate which controller sent them.

The controller is built for an ESP32-C6, receiver A for an ESP32-C3, and
receiver L for a generic ESP32. Receiver L defaults to GPIO 5, 33 LEDs per
logical zone, WS2811, and RGB color order; change those constants at the top of
its sketch to match the actual strip.
