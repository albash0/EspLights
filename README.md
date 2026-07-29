# ESP Lights

This repository contains two firmware variants:

- `encrypted/`: AES-128-CCMP encrypted ESP-NOW unicast with fixed peers and replay/duplicate rejection.
- `unencrypted/`: the original unencrypted ESP-NOW broadcast implementation.

## Encrypted setup

Copy each `secrets.example.h` to `secrets.h`. All controllers and the receiver
share one random 16-byte PMK. Give each controller a unique random 16-byte LMK,
then add its Wi-Fi station MAC and LMK to the receiver's `CONTROLLER_PEERS`
array. Each controller uses the receiver MAC and its own LMK.

The receiver sends telemetry separately to every configured controller. Replay
state is tracked independently per controller, so equal sequence numbers from
different controllers do not conflict. If multiple controllers send settings,
the most recent valid command wins.

The installed ESP32 Arduino core supports up to six encrypted peers. The
receiver build stops with a clear compile-time error if the configured list is
larger.

The unencrypted broadcast variant already accepts commands from multiple
controllers, but it cannot authenticate which controller sent them.

The controller is built for an ESP32-C6. The receiver is built for an ESP32-C3.
