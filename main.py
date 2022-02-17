import time
from sys import platform
import threading
from uartlib_common import Uart
import serial
import os
import sys
import glob
import keyboard
import argparse
import subprocess
import atexit


def main():
    fancy_menu = False

    def serial_ports():
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
            # from consolemenu.items import FunctionItem, SubmenuItem, CommandItem
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

    def show_menu(title, options):
        if fancy_menu:
            terminal_menu = TerminalMenu(
                options, title=title, clear_screen=True)
            menu_entry_index = terminal_menu.show()
            return menu_entry_index
        else:
            menu = SelectionMenu(options, title, show_exit_option=False)
            menu.show()
            menu.join()
            return menu.selected_option

    parser = argparse.ArgumentParser(description='Cli Kalman operation software.')
    parser.add_argument('--port', help='Specify port')

    args = parser.parse_args()

    if (platform == "linux" or platform == "linux2"):
        if(os.geteuid() != 0):
            print("Running with sudo")
            os.system('sudo ' + 'python ' + ' '.join(sys.argv))
            time.sleep(1)
            sys.exit()

        from simple_term_menu import TerminalMenu
        fancy_menu = True
        def clear(): os.system('clear')
    elif(platform == "darwin"):
        from simple_term_menu import TerminalMenu
        fancy_menu = True
        # OS X
        pass
    elif (platform == "win32"):
        from consolemenu import ConsoleMenu, SelectionMenu
        def clear(): return os.system('cls')
    title = "[ Nie podłączony ]"
    if(args.port != None):
        port = args.port
    else:
        while True:
            options = ["Wyszukaj dostępne porty", "Połącz manualnie"]+['Wyjdź']

            menu_entry_index = show_menu(title, options)
            # print(f"You have selected {options[menu_entry_index]}!")
            if(menu_entry_index == 0):
                # print(serial_ports())
                ports = serial_ports()
                options = ports+['Wyjdź']
                menu_entry_index = show_menu(title, options)
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
        options = ["AutoTest", "Manualna Jazda", "Stary Manipulator", "Nowy Manipulator"]+['Wyjdź']
        menu_entry_index = show_menu(title, options)
        if(menu_entry_index == 0):
            keep_running = True
            action_length = 60
            maxspeed = 30
            maxturn = 60
            while(keep_running):
                for i in range(action_length):
                    dir = 1 if i >= action_length/2 else -1
                    uart.MotorSetWheels2(0, maxturn*dir)
                    if keyboard.is_pressed("="):
                        action_length += 10
                    if (platform == "win32"):
                        if keyboard.is_pressed("-"):
                            action_length -= 10
                    else:
                        if keyboard.is_pressed("minus"):
                            action_length -= 10
                    if keyboard.is_pressed("]"):
                        maxturn += 10
                    if keyboard.is_pressed("["):
                        maxturn -= 10
                    if keyboard.is_pressed("enter") or keyboard.is_pressed("space"):
                        keep_running = False
                        break
                    if keyboard.is_pressed("escape"):
                        keep_running = False
                        break

                    action_length = max(action_length, 10)
                    maxturn = min(90, max(maxturn, 0))
                    print("SKRETY")
                    print("Dostępne klawisze: esc spacja/enter - + [ ]")
                    print("Last packet: [{:.1f}]".format(
                        time.time() - uart.last_rx_time))
                    print("[{:4d}][{:4d}]".format(action_length, maxturn))
                    time.sleep(0.1 - ((time.time() - starttime) % 0.1))
                    clear()

            uart.MotorSetWheels2(0, 0)
            time.sleep(2)

            keep_running = True
            action_length = 20

            while(keep_running):
                for i in range(action_length):
                    dir = 1 if i < action_length/4 else -1 if 2*action_length/4 <= i < 3*action_length/4 else 0
                    uart.MotorSetWheels2(maxspeed*dir, 0)
                    if keyboard.is_pressed("="):
                        action_length += 10
                    if (platform == "win32"):
                        if keyboard.is_pressed("-"):
                            action_length -= 10
                    else:
                        if keyboard.is_pressed("minus"):
                            action_length -= 10
                    if keyboard.is_pressed("]"):
                        maxspeed += 10
                    if keyboard.is_pressed("["):
                        maxspeed -= 10
                    if keyboard.is_pressed("enter") or keyboard.is_pressed("space"):
                        keep_running = False
                        break
                    if keyboard.is_pressed("escape"):
                        keep_running = False
                        break

                    action_length = max(action_length, 10)
                    maxturn = min(100, max(maxspeed, 0))

                    print("NAPEDY")
                    print("Dostępne klawisze: esc spacja/enter - + [ ]")
                    print("Last packet: [{:.1f}]".format(
                        time.time() - uart.last_rx_time))
                    print("[{:4d}][{:4d}]".format(action_length, maxturn))
                    time.sleep(0.1 - ((time.time() - starttime) % 0.1))
                    clear()

        elif(menu_entry_index == 1):
            keep_running = True
            maxspeed = 30
            maxturn = 30
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
                if keyboard.is_pressed("="):  # +
                    maxspeed += 5
                if (platform == "win32"):
                    if keyboard.is_pressed("-"):
                        maxspeed -= 5
                else:
                    if keyboard.is_pressed("minus"):
                        maxspeed -= 5
                if keyboard.is_pressed("]"):
                    maxturn += 10
                if keyboard.is_pressed("["):
                    maxturn -= 10
                if keyboard.is_pressed("escape"):
                    keep_running = False

                maxspeed = min(100, max(maxspeed, 0))
                maxturn = min(90, max(maxturn, 0))
                uart.MotorSetWheels2(speed, turn)

                print("Dostępne klawisze: esc ← ↓ ↑ → - + [ ]")
                print("Last packet: [{:.1f}]".format(
                    time.time() - uart.last_rx_time))
                print("[{:4d}][{:4d}]".format(maxspeed, maxturn))
                print("[{:4d}][{:4d}]".format(speed, turn))
                time.sleep(0.1 - ((time.time() - starttime) % 0.1))
                clear()
        elif(menu_entry_index == 2):
            keep_running = True
            local_arm_pos = None
            arm_seg = 0
            limits = [(a[0]/b, a[1]/b) for a, b in zip([(0, 270), (0, 0.1), (0, 0.043), (0, 0.079), (2, 360), (-1000, 1000)], [0.1, 0.0001, 0.0001, 0.0001, 0.1, 1.0])]
            keybinds = [('1', 'q'), ('2', 'w'), ('3', 'e'), ('4', 'r'), ('5', 't'), ('6', 'y')]
            while(keep_running):

                if not local_arm_pos:
                    if uart.old_arm_pos:
                        local_arm_pos = uart.old_arm_pos
                    else:
                        print("brak synchronizacji - esc aby wyjść")
                else:
                    for i, key in enumerate(keybinds):
                        if keyboard.is_pressed(key[0]):
                            local_arm_pos[i] += 5
                        if keyboard.is_pressed(key[1]):
                            local_arm_pos[i] -= 5
                    if keyboard.is_pressed("right"):
                        arm_seg += 1
                    if keyboard.is_pressed("left"):
                        arm_seg -= 1
                    if keyboard.is_pressed("up"):
                        local_arm_pos[arm_seg] += 5
                    if keyboard.is_pressed("down"):
                        local_arm_pos[arm_seg] -= 5
                    arm_seg = max(0, min(arm_seg, 5))
                    print("Dostępne klawisze: esc ← ↓ ↑ →")
                    print("Ostatnia ramka: [{:.1f}]".format(
                        time.time() - uart.last_rx_time))
                    print("[{:4d}][{:4d}][{:4d}][{:4d}][{:4d}][{:4d}]".format(*uart.old_arm_pos))
                    selected = ""
                    for i in range(0, 6):
                        selected += " ---- " if i == arm_seg else "      "
                    print(selected)
                    print(local_arm_pos)
                    local_arm_pos = [min(a[1], max(b, a[0])) for a, b in zip(limits, local_arm_pos)]
                    print(local_arm_pos)
                    uart.ArmSetPos(local_arm_pos)

                if keyboard.is_pressed("escape"):
                    keep_running = False
                # print("[{:4d}][{:4d}]".format(maxspeed, maxturn))
                # print("[{:4d}][{:4d}][{:4d}][{:4d}][{:4d}][{:4d}]".format(speed, turn))

                time.sleep(0.1 - ((time.time() - starttime) % 0.1))
                clear()
        elif(menu_entry_index == 3):
            keep_running = True
            arm_seg = 0
            max_torque = 20
            max_speed = 20
            keybinds = [('1', 'q'), ('2', 'w'), ('3', 'e'), ('4', 'r'), ('5', 't'), ('6', 'y')]
            while(keep_running):
                local_arm_pos = [0, 0, 0, 0, 0, 0]
                for i, key in enumerate(keybinds):
                    if keyboard.is_pressed(key[0]):
                        local_arm_pos[i] = max_speed
                    if keyboard.is_pressed(key[1]):
                        local_arm_pos[i] = -max_speed

                if keyboard.is_pressed("right"):
                    arm_seg += 1
                if keyboard.is_pressed("left"):
                    arm_seg -= 1
                if keyboard.is_pressed("up"):
                    local_arm_pos[arm_seg] = max_speed
                if keyboard.is_pressed("down"):
                    local_arm_pos[arm_seg] = -max_speed
                if keyboard.is_pressed("="):  # +
                    max_speed += 5
                if (platform == "win32"):
                    if keyboard.is_pressed("-"):
                        max_speed -= 5
                else:
                    if keyboard.is_pressed("minus"):
                        max_speed -= 5
                if keyboard.is_pressed("]"):
                    max_torque += 5
                if keyboard.is_pressed("["):
                    max_torque -= 5
                max_speed = min(100, max(max_speed, 0))
                max_torque = min(100, max(max_torque, 0))
                arm_seg = max(0, min(arm_seg, 5))
                print("Dostępne klawisze: esc ← ↓ ↑ → - +")
                print("Ostatnia ramka: [{:.1f}]".format(
                    time.time() - uart.last_rx_time))
                print("[{:4d}][{:4d}][{:4d}][{:4d}][{:4d}][{:4d}]SPEED[{:4d}]TORQUE[{:4d}]".format(*local_arm_pos, max_speed, max_torque))
                selected = ""
                for i in range(0, 6):
                    selected += " ---- " if i == arm_seg else "      "
                print(selected)
                uart.NewArmSetVel([*[10*pos for pos in local_arm_pos], max_torque])

                if keyboard.is_pressed("escape"):
                    local_arm_pos = [0, 0, 0, 0, 0, 0]
                    uart.NewArmSetVel([*local_arm_pos, max_torque])
                    keep_running = False

                time.sleep(0.1 - ((time.time() - starttime) % 0.1))
                clear()
        else:
            uart.stop()
            sys.exit()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
