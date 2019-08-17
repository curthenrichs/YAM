#!/usr/bin/env python3

## Note need to chmod u+x <file>

from inputs import get_gamepad, devices

import threading
import serial
import json
import time

EOL = b'\0'
PORT = "/dev/ttyACM0"

cartisian = {"x": 0, "y": 0}
gamepad_thread_alive = True

def _gamepad_thread():
    while gamepad_thread_alive:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)
            
            if event.code == "ABS_X":
                cartisian["x"] = event.state
            elif event.code == "ABS_Y":
                cartisian["y"] = event.state
            
def _print_gamepads():
    print("Gamepads")
    for d in devices.gamepads:
        print(d)

def _serial_msg():
    tx_msg = {
        "cartisian_move": cartisian,
        "heartbeat": True
    }
    
    tx_str = json.dumps(tx_msg)
    print(tx_str)
    
    ser.write(tx_str.encode("utf-8"))
    ser.write(EOL)
    

if __name__ == "__main__":
    
    _print_gamepads()
    
    # serial port connect
    ser = serial.Serial(PORT)
    print(ser.name)
    
    # gamepad thread
    t = threading.Thread(target=_gamepad_thread)
    t.start()
    
    # "main" publisher thread
    try:
        while True:
            time.sleep(1)
            _serial_msg()
    except:
        # close serial
        try:
            ser.close()
        except:
            print("Serial port closed with exception")
        
        # join gamepad thread
        gamepad_thread_alive = False
        try:
            t.join(0.5)
        except:
            print("Gamepad thread closed with exception")
