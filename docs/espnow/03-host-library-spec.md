# Host Library Spec: the `bugbot` Python package and scripts

Runs on the classroom PC. Talks to the dongle over USB serial; the dongle relays
to robots over ESP-NOW. **Implements:** the host side of
`docs/espnow/00-protocol.md`.

**Target:** Python 3.9+. **Only dependency:** `pyserial`, vendored into the
package so schools need no `pip` and no network (students copy a folder, plug in,
run).

## Package layout

```
bugbot/
  __init__.py     # exports Robot, connect
  protocol.py     # pack/unpack 00-protocol.md packets (struct). Single source of truth.
  transport.py    # dongle serial link: find port by USB VID/PID, framing, rx loop
  roster.py       # load/save the roster file, debounced auto-purge
  robot.py        # the Robot class: the student-facing, turtle-like API
  pickers.py      # optional Tkinter robot picker (Blink + Connect)
  _vendor/serial/ # vendored pyserial
scripts/
  bugbot_init.py  # provisioning over USB
```

## Roster file

Location: `%APPDATA%/BugBot/robots.json` (Windows) or `~/.bugbot/robots.json`.
The list of robots THIS PC owns:
```json
{
  "robots": {
    "<device_id hex>": { "key": "<pairing_key hex>", "name": "FROG-14", "last_mac": "aabbccddeeff" }
  }
}
```
Optionally shareable across class PCs for pooled robots (stable `device_id` makes
this safe). Students never see ids or keys.

## Provisioning: `scripts/bugbot_init.py`

1. Open the ROBOT's USB serial port (auto-detect the XIAO S3 by VID/PID, else prompt).
2. Send `espnow_init\n`.
3. Read the `ESPNOW_INIT id=.. key=.. mac=.. ch=.. fw=..` line.
4. Upsert into the roster keyed by `device_id`. If the id already exists, UPDATE its
   key/mac (do not duplicate). Assign or keep a friendly name.
5. Print confirmation. The robot reboots into ESP-NOW mode.

`bugbot_init.py --regenerate` sends `regenerate_key` instead (rotate the key,
keep the identity).

## Transport: `bugbot/transport.py`

- Auto-find the DONGLE serial port by its USB VID/PID (the C3).
- Implement the dongle framing from the dongle brief.
- `send_espnow(packet_bytes)` and a background receive loop delivering
  `(packet, rssi, mac)` to a queue/callback.

## Connection flow (`Robot.__init__` / `connect()`)

1. Load the roster. Pick a target: explicit `device_id` arg, a saved "last used",
   or raise the Tkinter picker listing roster robots plus any live ones heard via
   BEACON, with **Blink** (sends BLINK using the stored key) and **Connect**.
2. Open the dongle (transport).
3. PROBE the chosen `device_id` (dongle broadcasts; robot replies PROBE_ACK; the
   dongle learns the MAC).
4. CLAIM, carrying the `pairing_key` from the roster:
   - CLAIM_ACK(OK, token): store the token, start the heartbeat thread.
   - CLAIM_ACK(DENIED): another host holds it; report "robot busy".
   - **AUTH_FAIL(BAD_KEY): this robot was re-paired away from us. Auto-purge (below).**
5. Return the live `Robot`.

## Heartbeat thread

Send HEARTBEAT (key + token) every ~2 s while connected. On `disconnect()` or
process exit (atexit + context manager), send RELEASE and stop.

## Auto-purge (`roster.py` + connection layer)

- Count CONSECUTIVE `AUTH_FAIL(BAD_KEY)` responses per `device_id`.
- After 2-3 in a row (debounce, so one corrupt packet cannot evict a robot you
  still own), remove that `device_id` from the roster and PRINT:
  `FROG-14 was re-paired to another host and removed from your list.`
- **Silence must NEVER purge.** No reply means the robot is off or out of range;
  keep it. Only an explicit AUTH_FAIL purges.

## Student-facing API (`robot.py`), turtle-like

```python
from bugbot import Robot
bot = Robot()                     # roster pick + connect (picker if needed)

# Motion  -> COMMAND DRIVE / DRIVE_VEC / STOP
bot.forward(50, distance=20)      # speed %, optional cm (blocking if given)
bot.backward(50); bot.left(40); bot.right(40)
bot.turn(90)                      # degrees, uses heading (blocking)
bot.drive(left, right)            # free-running tank, non-blocking
bot.stop()

# Output
bot.led(255, 0, 0)  ;  bot.led("red")
bot.servo(1, 90)
bot.beep(2000, 200)

# Sensors -> COMMAND READ -> RESPONSE
bot.distance()                    # cm
bot.heading()                     # deg
bot.position()                    # (x, y) cm, odometry
bot.battery()                     # %

# bot.photo()  -> NotImplemented for now (PHOTO is stubbed)

bot.disconnect()                  # or:  with Robot() as bot: ...
```

Units: speed 0-100 (%), distance cm, angle degrees. Clamp on the host AND the
firmware.

## Notes

- `protocol.py` is the single packing source and must match `00-protocol.md` and
  the firmware/dongle headers exactly.
- All auth (keys, tokens) is internal. Students only ever see `Robot` and its
  turtle-like methods.
