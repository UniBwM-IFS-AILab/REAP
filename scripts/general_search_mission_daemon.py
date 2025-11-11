#!/usr/bin/env python3
import msvcrt
import os
import time

file_path = 'help.txt'

def send_mission():
    print('sending mission.')
    # the search mission can be sent to the planning and execution framework here.


def clear_help_file():
    print('heard the call.')
    with open(file_path, 'w') as file:
        msvcrt.locking(file.fileno(), msvcrt.LK_LOCK, 1)
        print('locking help file.')
        msvcrt.locking(file.fileno(), msvcrt.LK_UNLCK, 1)
        print('cleared file. released the lock.')

def main():
    try:
        print('listening fo help calls.')
        while True:
            if os.path.exists(file_path) and os.path.getsize(file_path) > 2:
                clear_help_file()
                send_mission()
            else:
                print('nothing to hear.')
            time.sleep(5)
    except Exception as e:
        print('stop listening: ' + str(e))

if __name__ == "__main__":
    main()
