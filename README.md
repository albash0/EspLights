# ESP Lights

This repository contains two firmware variants:

- `encrypted/`: AES-128-CCMP encrypted ESP-NOW unicast with fixed peers and replay/duplicate rejection.
- `unencrypted/`: the original unencrypted ESP-NOW broadcast implementation.

## Encrypted setup

Copy each `secrets.example.h` to `secrets.h`, then set the peer MAC address and the same random 16-byte PMK and LMK in both files. Live `secrets.h` files are excluded from Git.

The controller is built for an ESP32-C6. The receiver is built for an ESP32-C3.
