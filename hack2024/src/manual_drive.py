#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import curses
from control import Control

drive = Control()

def main(stdscr):
    steer = 0
    speed = 0
    stdscr.timeout(100)  # 100ms 후에 키 입력이 없으면 -1을 반환

    while True:
        c = stdscr.getch()
  
        if c == curses.KEY_LEFT:
            steer = max(-1, steer - 0.1)
        elif c == curses.KEY_RIGHT:
            steer = min(1, steer + 0.1)
        elif c == curses.KEY_UP:
            speed = min(100, speed + 10)
        elif c == curses.KEY_DOWN:
            speed = max(-100, speed - 10)
        elif c == -1:
            pass
        elif c == 47:
            steer = 0
        else:
            pass

        print(steer, speed)
        
        # move car
        drive.run(-steer, speed)
   

curses.wrapper(main)
