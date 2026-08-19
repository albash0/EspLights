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
commands. In Music mode, receiver L asks receiver A for its processed microphone
envelope and uses that wireless level with L's own brightness and sensitivity
settings. A sends only the normalized level (not raw audio), about 30 times per
second, and stops when L leaves Music mode. If A, its microphone, or the direct
link is unavailable, L automatically keeps working with a synthetic pulse.
Automatic presence control applies only to receiver A.

## Encrypted setup

Copy each `secrets.example.h` to `secrets.h`. All controllers and the receiver
share one random 16-byte PMK. Give every controller/receiver pair a unique
random 16-byte LMK. Each receiver lists its controllers in `CONTROLLER_PEERS`.
Each controller lists receiver A and receiver L in `RECEIVER_PEERS`, including
the receiver ID, station MAC, and matching LMK.

The direct A-to-L Music link uses its own LMK. In both receiver `secrets.h`
files, set `RECEIVER_LINK_CONFIGURED` to `1`, use L's station MAC as
`LIGHT_RECEIVER_ADDRESS` on A, use A's station MAC as
`PRIMARY_RECEIVER_ADDRESS` on L, and copy the same random
`RECEIVER_LINK_LMK` to both. Receiver A currently reports its station MAC as
`9C:CC:01:C0:77:4C`; receiver L prints its station MAC at startup. Link packets
have independent boot sessions and monotonically increasing sequence numbers,
so duplicated or out-of-order requests and music levels are rejected.

Receiver A sends telemetry separately to every configured controller. Replay
state is tracked independently per controller, so equal sequence numbers from
different controllers do not conflict. If multiple controllers send settings,
the most recent valid command wins.

The installed ESP32 Arduino core supports up to six encrypted peers per device.
The direct receiver link uses one of those peer slots when enabled. Builds stop
with a clear compile-time error if a configured list is larger.

The unencrypted broadcast variant already accepts commands from multiple
controllers and discovers the A-to-L Music link without secrets. It rejects
repeated link sequence numbers, but it cannot authenticate which device sent a
packet.

The controller is built for an ESP32-C6, receiver A for an ESP32-C3, and
receiver L for a generic ESP32. Receiver L defaults to GPIO 5, 33 LEDs per
logical zone, WS2811, and RGB color order; change those constants at the top of
its sketch to match the actual strip.
