import os
import sys
import tty
import termios


def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # print(ch, ord(ch))
    return ord(ch)


def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if c1 == 0x03:
        exit()
    if c1 != 0x1b:
        return c1
    c2 = getchar()
    c3 = getchar()
    return c2*100+c3
    # sys.stdin.flush()


class KEY2:
    ArrowLeft = 9168
    ArrowUp = 9165
    ArrowRight = 9167
    ArrowDown = 9166
    Enter = 13
    Memu = 7980  # KeyM (Menu)
    Back = 7981  # KeyB (Back)
    Run = 7982  # KeyR (Run)
    Stop = 2727  # KeyS (Stop or Home)
