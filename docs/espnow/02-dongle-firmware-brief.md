# Dongle Firmware Brief: serial-to-ESP-NOW bridge

**New firmware** for a XIAO ESP32-C3 (native USB, cheapest XIAO with a radio),
housed in a 3D-printed case and plugged into a classroom PC.
**Implements:** the dongle side of `docs/espnow/00-protocol.md`.

## Role

A thin, almost stateless bridge. All session logic (claim, heartbeat, auth,
auto-purge) lives in the HOST library, NOT here. The dongle only relays packets
and resolves addressing.

- **PC to dongle (USB serial):** the host sends framed packets; the dongle
  transmits them over ESP-NOW.
- **robot to dongle (ESP-NOW):** the dongle forwards received packets to the PC
  over serial, tagged with RSSI and sender MAC.
- The dongle keeps a small `device_id -> MAC` table so it can unicast. It learns
  MACs from any received packet (ESP-NOW gives the sender MAC), so PROBE_ACK and
  BEACON both populate it.

## USB serial framing (PC <-> dongle)

A simple framed binary protocol so both ends agree:

- Frame: `0x7E` start, `len`(2, LE), `payload`(len), `crc16`(2). Byte-stuff
  `0x7E`/`0x7D` in the payload, or use COBS. (Pick one and document it.)
- `payload[0]` = opcode:
  - `0x01 SEND` (PC to dongle): followed by a full ESP-NOW packet (per
    `00-protocol.md`) to transmit. The dongle reads `device_id` from the packet
    header, looks it up in the MAC table, and unicasts; if unknown (or a
    broadcast type like PROBE), it broadcasts.
  - `0x02 RECV` (dongle to PC): an ESP-NOW packet that was received, followed by
    1 byte RSSI and 6 bytes sender MAC.
  - `0x10 SET_CHANNEL` (PC to dongle): 1 byte channel.
  - `0x11 INFO` (dongle to PC): firmware version + channel, sent on connect.
- Native USB CDC, so the baud rate is nominal.

## Behaviour

- On boot: WiFi STA mode (no connect), set channel, `esp_now_init`, register the
  broadcast peer, send INFO to the PC.
- On SEND: extract `device_id` from the header. Known MAC -> `esp_now_add_peer`
  (once) + unicast. Unknown, or a broadcast-type packet -> broadcast.
- On ESP-NOW receive: read the sender MAC and the packet's `device_id`, update the
  MAC table, wrap as RECV (packet + RSSI + MAC), send to the PC.
- Stay stateless beyond the MAC table. No auth, no claim, no lease.

## Deliverables

- The bridge firmware + the shared protocol header (matching `00-protocol.md`).
- A self-test mode: echo serial, plus a "scan" that broadcasts PROBE and prints
  PROBE_ACKs, so the radio and the serial link can be validated independently of
  the host library.
