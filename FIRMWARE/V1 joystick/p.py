#!/usr/bin/env python3
"""
Smooth Delta gamepad controller (pygame) — adjustable SPEED variable.

SPEED affects:
 - XY velocity
 - Z velocity
 - update frequency
"""

import sys
import threading
import time
import serial
import pygame
import re

# ---------------- USER SPEED CONTROL ----------------
SPEED = 1.0      # 1.0 = normal, 0.5 = slower, 2.0 = faster, 3.0 = very fast
# -----------------------------------------------------

# ---------------- BASE CONFIG ----------------
SERIAL_PORT = "COM5"
BAUDRATE = 115200

SEND_INTERVAL_BASE = 0.04     # will be divided by SPEED
AXIS_MAX_VEL_BASE = 150.0      # mm/s (will be multiplied by SPEED)
Z_MAX_VEL_BASE = 60.0         # mm/s (will be multiplied by SPEED)

DEADZONE = 0.08
INVERT_Y = True

# initial estimated position
estX = 0.0
estY = 0.0
estZ = -150.0

# Buttons
BUTTON_UP = 0
BUTTON_DOWN = 1
BUTTON_PROMPT = 2
BUTTON_EXIT = 3

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
# -----------------------------------------------------

# Derived from SPEED
SEND_INTERVAL = SEND_INTERVAL_BASE / SPEED
AXIS_MAX_VEL = AXIS_MAX_VEL_BASE * SPEED
Z_MAX_VEL = Z_MAX_VEL_BASE * SPEED

ser = None
stop_event = threading.Event()
pos_lock = threading.Lock()

_est_pos = {"x": estX, "y": estY, "z": estZ}

moved_re = re.compile(r"Moved to:\s*([+-]?\d+(\.\d*)?)\s+([+-]?\d+(\.\d*)?)\s+([+-]?\d+(\.\d*)?)")

def open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print(f"[serial] opened {SERIAL_PORT} @ {BAUDRATE}")
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print("[serial] failed:", e)
        sys.exit(1)

def send_line(s: str):
    if not s.endswith("\n"):
        s += "\n"
    try:
        ser.write(s.encode("utf-8"))
    except:
        pass

def serial_reader():
    global _est_pos
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print("[arduino]", line)
                m = moved_re.search(line)
                if m:
                    try:
                        x = float(m.group(1))
                        y = float(m.group(3))
                        z = float(m.group(5))
                        with pos_lock:
                            _est_pos["x"] = x
                            _est_pos["y"] = y
                            _est_pos["z"] = z
                    except:
                        pass
        except:
            time.sleep(0.02)

def prompt_coords():
    try:
        s = input("coords> ").strip()
        if s:
            send_line(s)
    except:
        pass

def main():
    global _est_pos

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("[joy] Controller:", joy.get_name())

    open_serial()

    threading.Thread(target=serial_reader, daemon=True).start()

    up_held = False
    down_held = False

    last_send = time.time()
    last_loop = time.time()

    print(f"[info] SPEED = {SPEED}")
    print(f"[info] XY speed = {AXIS_MAX_VEL} mm/s")
    print(f"[info] Z speed  = {Z_MAX_VEL} mm/s")
    print(f"[info] Send interval = {SEND_INTERVAL}s")
    print("[info] Ready. Use stick for XY, hold A/B for Z, X for coords, Y exit.")

    try:
        while True:
            now = time.time()
            dt = now - last_loop
            last_loop = now

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    raise KeyboardInterrupt()

                elif ev.type == pygame.JOYBUTTONDOWN:
                    if ev.button == BUTTON_UP:
                        up_held = True
                    elif ev.button == BUTTON_DOWN:
                        down_held = True
                    elif ev.button == BUTTON_PROMPT:
                        threading.Thread(target=prompt_coords, daemon=True).start()
                    elif ev.button == BUTTON_EXIT:
                        raise KeyboardInterrupt()

                elif ev.type == pygame.JOYBUTTONUP:
                    if ev.button == BUTTON_UP:
                        up_held = False
                    elif ev.button == BUTTON_DOWN:
                        down_held = False

            # joystick axes
            try:
                ax = float(joy.get_axis(LEFT_STICK_X))
                ay = float(joy.get_axis(LEFT_STICK_Y))
            except:
                ax = ay = 0.0

            if INVERT_Y:
                ay = -ay

            if abs(ax) < DEADZONE: ax = 0
            if abs(ay) < DEADZONE: ay = 0

            vx = ax * AXIS_MAX_VEL
            vy = ay * AXIS_MAX_VEL
            vz = (Z_MAX_VEL if up_held else -Z_MAX_VEL if down_held else 0)

            if now - last_send >= SEND_INTERVAL:
                with pos_lock:
                    _est_pos["x"] += vx * (now - last_send)
                    _est_pos["y"] += vy * (now - last_send)
                    _est_pos["z"] += vz * (now - last_send)

                    tx = _est_pos["x"]
                    ty = _est_pos["y"]
                    tz = max(-350, min(-10, _est_pos["z"]))  # safety clamp

                    _est_pos["z"] = tz

                send_line(f"{tx:.3f} {ty:.3f} {tz:.3f}")
                last_send = now

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("[info] Exiting...")
    finally:
        stop_event.set()
        time.sleep(0.05)
        if ser:
            ser.close()
        pygame.joystick.quit()
        pygame.quit()

if __name__ == "__main__":
    main()
