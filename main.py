import time
from sys import platform
import threading
from uartlib_common import Uart
import serial
import os
import sys
import glob
import keyboard
from simple_term_menu import TerminalMenu
import argparse
import subprocess
import atexit


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


parser = argparse.ArgumentParser(description='Cli Kalman operation software.')
parser.add_argument('--port', help='Specify port')

args = parser.parse_args()


if (platform == "linux" or platform == "linux2"):
    if(os.geteuid() != 0):
        print("Running with sudo")
        os.system('sudo ' + 'python ' + ' '.join(sys.argv))
        time.sleep(1)
        sys.exit()

    def clear(): os.system('clear')
elif(platform == "darwin"):
    # OS X
    pass
elif (platform == "win32"):
    def clear(): return os.system('cls')
title = "[ Nie podłączony ]"
if(args.port != None):
    port = args.port
else:
    while True:
        options = ["Wyszukaj dostępne porty", "Połącz manualnie"]+['Wyjdź']
        terminal_menu = TerminalMenu(
            options, title=title, clear_screen=True)
        menu_entry_index = terminal_menu.show()
        # print(f"You have selected {options[menu_entry_index]}!")
        if(menu_entry_index == 0):
            # print(serial_ports())
            ports = serial_ports()
            options = ports+['Wyjdź']
            terminal_menu = TerminalMenu(options, title=title)
            menu_entry_index = terminal_menu.show()
            if(menu_entry_index != len(ports)):
                port = ports[menu_entry_index]
                break
        elif(menu_entry_index == 1):
            port = input("Podaj nazwę portu: ")
            break
        else:
            sys.exit()


uart = Uart(PORTNAME=port, PRINT_TX=False, PRINT_RX=False)

uart.start()
atexit.register(uart.stop)
title = "[ {} ]".format(port)


starttime = time.time()
maxspeed = 30
maxturn = 30


while True:
    options = ["AutoTest", "Manualna Jazda", "Test Manipulatora"]+['Wyjdź']
    terminal_menu = TerminalMenu(
        options, title=title, clear_screen=True)
    menu_entry_index = terminal_menu.show()
    if(menu_entry_index == 1):
        keep_running = True
        while(keep_running):
            speed = 0
            turn = 0
            if keyboard.is_pressed("up"):
                speed = maxspeed
            if keyboard.is_pressed("down"):
                speed = -maxspeed
            if keyboard.is_pressed("right"):
                turn = maxturn
            if keyboard.is_pressed("left"):
                turn = -maxturn
            if keyboard.is_pressed("plus"):
                maxspeed += 5
            if keyboard.is_pressed("minus"):
                maxspeed -= 5
            if keyboard.is_pressed("]"):
                maxturn += 5
            if keyboard.is_pressed("["):
                maxturn -= 5
            if keyboard.is_pressed("escape"):
                keep_running = False
            uart.MotorSetWheels2(speed, turn)
            print("Last packet: [{:.1f}]".format(
                time.time() - uart.last_rx_time))
            print("[{:4d}][{:4d}]".format(maxspeed, maxturn))
            print("[{:4d}][{:4d}]".format(speed, turn))
            time.sleep(0.1 - ((time.time() - starttime) % 0.1))
            clear()
    elif(menu_entry_index == 1):
        port = input("Podaj nazwę portu: ")
        break
    else:
        uart.stop()
        sys.exit()
