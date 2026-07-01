# Assembly

These steps apply to both the **full kit** and the **board-only** build. If you bought the kit, all parts are already in the box. If you built from a board, make sure you have everything from the [parts list](parts-list.md) and your [3D-printed parts](3d-printing.md) ready before starting.

**Time required:** approximately 30–45 minutes.  
**Tools:** small Phillips screwdriver.  
**Soldering required:** none.

---

!!! note "Full assembly guide coming soon"
    Step-by-step photos and instructions are being prepared. The sections below outline the build sequence — detailed steps and images will be added for each stage.

---

## Stage 1 — Motor mounts

_Attach the four motors to the 3D-printed motor mounts, then secure the mounts to the base plate._

<!-- TODO: steps + photos -->

---

## Stage 2 — Drive system

_Attach the drive components to the motor shafts._

<!-- TODO: steps + photos -->

---

## Stage 3 — PCB

_Mount the BugBot carrier PCB onto the base plate standoffs. Seat the XIAO ESP32-S3 Sense into its socket on the PCB._

<!-- TODO: steps + photos, board orientation note -->

---

## Stage 4 — Sensors

_Mount the VL53L5CX ToF and the camera bracket to the front of the chassis. Route the cables to the PCB._

<!-- TODO: steps + photos, cable routing note -->

---

## Stage 5 — Battery

_Connect the battery to the PCB power connector. Tuck the battery into the chassis and secure with the strap._

<!-- TODO: steps + photos, polarity warning -->

---

## Stage 6 — Top cover (optional)

_Clip or screw the top cover onto the chassis._

<!-- TODO: steps + photos -->

---

## Stage 7 — Power on and test

1. Power the robot on using the switch on the PCB.
2. The RGB LED should flash briefly during boot.
3. Plug the dongle into your PC.
4. Follow the [Getting Started](../getting-started.md) guide to provision the robot and run a drive test.

---

## Troubleshooting assembly issues

**Robot does not power on**  
Check the battery connector polarity and make sure the PCB power switch is in the ON position.

**Motors spin but robot does not move straight**  
Check that all motor connectors are plugged into the correct ports and that the motor polarity is correct.

**LED does not light up on boot**  
Check that the XIAO ESP32-S3 Sense is fully seated in its socket on the PCB.
