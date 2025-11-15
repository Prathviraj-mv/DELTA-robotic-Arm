#!/usr/bin/env python3
"""
Delta robot gamepad controller using pygame.

Maps a gamepad's left stick to XY motion and buttons to up/down.
Sends single-line commands to Arduino (same format as your sketch):
  - 'w','a','s','d' (letters) for small XY steps (repeat while held)
  - uppercase letters (W/A/S/D) are treated by Arduino as large steps if you hold shift;
    here we only send lowercase but you can modify to send uppercase when a modifier button held.
  - ' ' (a single space + newline) -> UP (Arduino interprets a space-only line)
  - 'z' -> DOWN
  - 't' -> type coordinates via console prompt
  - 'ESC' -> exit (mapped to a button)
"""

import sys
import threading
import time
import serial
import pygame

# ----- CONFIG -----
SERIAL_PORT = "COM5"          # <<-- set your serial port
BAUDRATE = 115200
SEND_BASE_INTERVAL = 0.08    # base interval (s) for minimum repeated command when stick at threshold
DEADZONE = 0.20              # joystick deadzone (0..1)
MAX_SEND_RATE = 0.01         # shortest delay between sends when stick fully deflected (s)
AXIS_MAX_MAG = 1.0           # joystick axis magnitude scale
BUTTON_UP = 0                # button index for UP (A on many controllers)
BUTTON_DOWN = 1              # button index for DOWN (B)
BUTTON_PROMPT = 2            # button index for prompt 't' (X)
BUTTON_EXIT = 3              # button index for exit (Y)
LEFT_STICK_X = 0             # axis index for left stick X
LEFT_STICK_Y = 1             # axis index for left stick Y
INVERT_Y = True              # set True if pushing up gives negative values on your controller (common)
# ------------------

ser = None
stop_event = threading.Event()

def open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print(f"[serial] opened {SERIAL_PORT} @ {BAUDRATE}")
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[serial] failed open {SERIAL_PORT}: {e}")
        sys.exit(1)

def send_line(s: str):
    """Send string terminated by newline."""
    if ser is None:
        return
    if not s.endswith("\n"):
        s = s + "\n"
    try:
        ser.write(s.encode("utf-8"))
    except Exception as e:
        print(f"[serial] write error: {e}")

def serial_reader():
    """Print any lines coming back from Arduino."""
    while not stop_event.is_set():
        try:
            if ser is None:
                time.sleep(0.05)
                continue
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"[arduino] {line}")
        except Exception:
            time.sleep(0.05)
    print("[serial] reader exiting")

def prompt_coords():
    """Prompt user to type absolute coords and send them to Arduino."""
    try:
        s = input("coords> ").strip()
    except EOFError:
        return
    if s:
        send_line(s)
        print(f"[sent] {s}")

def clamp(v, a, b):
    return a if v < a else (b if v > b else v)

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick/gamepad detected. Plug one in and restart.")
        return

    # Use the first joystick
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[joy] Using joystick: {joy.get_name()} (axes={joy.get_numaxes()} buttons={joy.get_numbuttons()})")

    open_serial()

    # start serial reader
    th = threading.Thread(target=serial_reader, daemon=True)
    th.start()

    last_send_time_x = 0.0
    last_send_time_y = 0.0
    clock = pygame.time.Clock()

    try:
        print("[info] Gamepad control started. Move left stick to drive XY. Buttons: A->UP, B->DOWN, X->prompt, Y->exit")
        while True:
            # handle events first (pygame requires pumping events)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    raise KeyboardInterrupt()
                elif ev.type == pygame.JOYBUTTONDOWN:
                    # map buttons
                    b = ev.button
                    if b == BUTTON_UP:
                        # send a space-only line => Arduino moves UP
                        send_line(" ")
                    elif b == BUTTON_DOWN:
                        send_line("z")
                    elif b == BUTTON_PROMPT:
                        # spawn prompt thread so input() doesn't block event loop / joystick handling
                        threading.Thread(target=prompt_coords, daemon=True).start()
                    elif b == BUTTON_EXIT:
                        print("[info] Exit button pressed.")
                        raise KeyboardInterrupt()

            # read axes
            ax_x = 0.0
            ax_y = 0.0
            try:
                ax_x = float(joy.get_axis(LEFT_STICK_X))
            except Exception:
                ax_x = 0.0
            try:
                ax_y = float(joy.get_axis(LEFT_STICK_Y))
            except Exception:
                ax_y = 0.0

            # optionally invert Y if needed
            if INVERT_Y:
                ax_y = -ax_y

            # apply deadzone
            if abs(ax_x) < DEADZONE:
                ax_x = 0.0
            if abs(ax_y) < DEADZONE:
                ax_y = 0.0

            now = time.time()
            # map axis magnitude to send interval (more deflection -> smaller delay -> faster repeat)
            def send_for_axis(axis_val, last_time, negative_cmd, positive_cmd):
                if axis_val == 0.0:
                    return last_time
                mag = clamp(abs(axis_val), 0.0, AXIS_MAX_MAG)
                # linear interpolation between SEND_BASE_INTERVAL and MAX_SEND_RATE
                interval = SEND_BASE_INTERVAL - ( (SEND_BASE_INTERVAL - MAX_SEND_RATE) * (mag / AXIS_MAX_MAG) )
                interval = max(MAX_SEND_RATE, interval)
                if now - last_time >= interval:
                    # choose direction based on sign
                    cmd = positive_cmd if axis_val > 0 else negative_cmd
                    send_line(cmd)
                    return now
                return last_time

            last_send_time_x = send_for_axis(ax_x, last_send_time_x, 'a', 'd')  # left->'a', right->'d'
            last_send_time_y = send_for_axis(ax_y, last_send_time_y, 's', 'w')  # down->'s', up->'w'

            # small sleep to avoid busy loop and keep pygame responsive
            clock.tick(120)  # limit to 120 Hz loop
    except KeyboardInterrupt:
        print("[info] quitting...")
    finally:
        stop_event.set()
        time.sleep(0.05)
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        pygame.joystick.quit()
        pygame.quit()
        print("[info] closed.")

if __name__ == "__main__":
    main()
