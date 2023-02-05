import serial
import time
import threading
import struct

#########################################################
# UARTlib v2
# Patryk Jarosz, 08.2019
##
# Ponizszy skrypt dostarcza funkcje do obslugi ramek zgodnych z
# protokolem UARTlib oraz obsluguje ich nadawanie i odbior przez
# zadany port szeregowy. Odbior zrealizowany jest na osobnym watku.
# Dodatkowo tworzony jest plik z logami (nadawanie i odbior).
##
# Uzytkowanie:
# start() - otwieranie portu, uruchamianie watku odbiorczego
# stop() - zamykanie portu, wylaczanie watku odbiorczego


#########################################################
#                   konfiguracja
class Uart:
    def __init__(self, PORTNAME='virtualcom0', PRINT_TX=False, PRINT_RX=False):
        self.PORTNAME = PORTNAME
        self.PRINT_TX = PRINT_TX
        self.PRINT_RX = PRINT_RX

    BAUDRATE = 38400
    ARGS_COUNT_BYTES = 1  # dlugosc pola ilosci argumentow (1 lub 2 bajty)
    # wylaczanie wysylania danych (samo generowanie i wyswietlanie)
    DISABLE_TX = False

    # wylaczenie dekodowania ramek odbiorczych, wyswietlane surowe dane przychodzace z urzadzenia
    DISABLE_DECODING = False
    PRINT_HEX = True  # wyswietlanie argumentow szesnastkowo
    DISABLE_RAW_RX = True

    START_BYTE = '<'  # nie zmieniac
    STOP_BYTE = '>'  # nie zmieniac

    RX_FRAMES_FILTER = (0x5F,)

    science = []

    #########################################################
    #      warstwa komunikacyjna - ramki na magistrali

    def CanCommonResetAll(self):
        data = [ord(i) for i in 'do it']
        self._generate(0x30, data)

    def CanCommonResetDevice(self, id):
        data = [ord(i) for i in 'do it']
        data.append(id)
        self._generate(0x31, data)

    def CanCommonSetPid(self, p, i, d, min, max, id=2, flags=1):
        data = [(p & 0xFFFF) >> 8, p & 0xFF, i, d, min, max, id, flags]
        self._generate(0x32, data)

    def CanCommonGetPidRequest(self, id=2):
        data = [id, ]
        self._generate(0x33, data)

    def CanArmSetPos(self, positions):
        if (len(positions) != 6):
            raise ValueError('6 elements required')
        data = []
        for pos in positions:
            data.append((pos & 0xFFFF) >> 8)
            data.append(pos & 0xFF)
        self._generate(0x20, data[:8]) + self.generate(0x21, data[8:12])

    def CanArmSetPos3(self, positions):
        if (len(positions) != 6):
            raise ValueError('6 elements required')
        data = []
        for pos in positions:
            data.append((pos & 0xFFFF) >> 8)
            data.append(pos & 0xFF)
        self._generate(0x21, data[8:12])

    def CanArmSetPos2(self, pos):
        data = []
        for i in range(6):
            data.append((pos & 0xFFFF) >> 8)
            data.append(pos & 0xFF)
        self._generate(0x20, data[:8]) + self.generate(0x21, data[8:12])

    def CanArmSetMagnet(self, state=True):
        data = [0, ]
        if (state):
            data[0] = 1
        self._generate(0x22, data)

    def CanArmCalibrate(self, joint):
        data = [ord(i) for i in 'do it']
        data.append(joint)
        self._generate(0x23, data)

    def CanArmDebugTx(self, data):
        data = data + [0, ]*(8 - len(data))
        self._generate(0x24, data)

    def CanArmDebugTxInfo(self, flags):
        self.CanArmDebugTx([1, flags, ])

    def CanArmDebugTxProtection(self, uvpen, uvpdis, ocp):
        data = [2, (uvpen & 0xFFFF) >> 8, uvpen & 0xFF,
                (uvpdis & 0xFFFF) >> 8, uvpdis & 0xFF, ocp]
        self.CanArmDebugTx(data)

    def CanArmDebugTxGripper(self, raw):
        data = [3, (raw & 0xFFFF) >> 8, raw & 0xFF] + [ord(i) for i in 'do it']
        self.CanArmDebugTx(data)

    def CanArmPoll(self):
        self._generate(0x27)

    def CanMotorSetWheels(self, speed, pos):
        if (len(speed) != 4):
            raise ValueError('wymagane 4 wartosci predkosci!')
        if (len(pos) != 4):
            raise ValueError('wymagane 4 wartosci pozycji!')

        data = []
        for i in speed:
            data.append(i & 0xFF)
        for i in pos:
            data.append(i & 0xFF)
        self._generate(0x10, data)

    def CanMotorSetWheels2(self, speed, pos):
        data = []
        for i in range(4):
            data.append((speed) & 0xFF)
        for i in range(4):
            data.append((pos) & 0xFF)
        self._generate(0x10, data)

    def CanMotorCalibrate(self, channel):
        data = [ord(i) for i in 'do it'] + [channel, ]
        self._generate(0x11, data)

    def CanMotorDebugTx(self, data):
        data = data + [0, ]*(8 - len(data))
        self._generate(0x12, data)

    def CanMotorDebugTxInfo(self, flags):
        self.CanMotorDebugTx([1, flags, ])

    def CanMotorDebugTxProtection(self, uvpen, uvpdis, ocp):
        data = [(uvpen & 0xFFFF) >> 8, uvpen & 0xFF,
                (uvpdis & 0xFFFF) >> 8, uvpdis & 0xFF, ocp]
        self.CanMotorDebugTx([2, ] + data)

    def CanMotorDebugTxSpeed(self, max_speed):
        self.CanMotorDebugTx([3, max_speed])

    def CanMotorPoll(self):
        self._generate(0x17)

    def CanPartySetIgniters(self, flags):
        self._generate(0x50, [flags, ])

    def CanPartySetAuto(self, flags):
        self._generate(0x51, [flags, ])

    def CanPartySetRgb(self, anim_type):
        data = [anim_type, ] + [0, ]*5
        self._generate(0x52, data)

    def CanPartySetPower(self, flags):
        self._generate(0x53, [flags, ])

    def CanPartyDebugTx(self, data):
        self._generate(0x54, data)

    def CanUniversalSetBridge(self, id, flags, values):
        if len(values) != 2:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append((i & 0xFFFF) >> 8)
            data.append(i & 0xFF)
        self._generate(0x80, data)

    def CanUniversalSetServo(self, id, flags, values):
        if len(values) != 4:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append(i & 0xFF)
        self._generate(0x81, data)

    def CanUniversalSetPwm(self, id, flags, values):
        if len(values) != 6:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append(i & 0xFF)
        self._generate(0x82, data)

    def CanUniversalDebugTx(self, data):
        data = data + [0, ]*(8 - len(data))
        self._generate(0x84, data)

    def CanUniversalDebugTxInfo(self, flags):
        self.CanUniversalDebugTx([1, flags, ])

    def CanUniversalDebugTxProtection(self, uvpen, uvpdis):
        data = [2, (uvpen & 0xFFFF) >> 8, uvpen & 0xFF,
                (uvpdis & 0xFFFF) >> 8, uvpdis & 0xFF]
        self.CanUniversalDebugTx(data)

    def CanUniversalDebugTxDefaultServo(self, flags, values):
        if len(values) != 4:
            raise ValueError('wymagane 4 wartosci pozycji!')
        for i in range(4):
            if (values[i]) > 100:
                values[i] = 100
            if (values[i]) < 0:
                values[i] = 0
        self.CanUniversalDebugTx([3, flags, ] + values)

    def CanSciencePoll(self, id):
        self._generate(0x90, [id, ])

    def CanScienceDebugTx(self, data):
        data = data + [0, ]*(8 - len(data))
        self._generate(0x91, data)

    def CanScienceDebugTxInfo(self, flags):
        self.CanScienceDebugTx([1, flags, ])

    def MkArmJointPositioningStart(self, jointId):
        data = [jointId]
        print(data)
        self._generate(0xBB, data)

    def MkArmJointPositioningAbort(self, jointId):
        data = [jointId]
        print(data)
        self._generate(0xBC, data)

    def MkArmTorque(self, jointId, torq):
        data = [jointId]
        x = list(struct.pack("<f", torq))
        for i in x:
            data.append(ord(i))
        print(data)
        self._generate(0xBF, data)

    def MkArmVel(self, jointId, vel, acc, torq):
        data = [jointId]
        x = list(struct.pack("<fff", torq, acc, vel))
        for i in x:
            data.append(ord(i))
        print(data)
        self._generate(0xC0, data)

    def MkArmPos(self, jointId, pos, vel, acc, torq):
        data = [jointId]
        x = list(struct.pack("<ffff", torq, acc, vel, pos))
        for i in x:
            data.append(ord(i))
        print(data)
        self._generate(0xC1, data)

    #########################################################
    # warstwa komunikacyjna - ramki protokolu 433/WiFi/Auto

    old_arm_pos = None

    def CommonResetAll(self):
        self._generate(0x00)

    def CommonResetDevice(self, id):
        self._generate(0x01, [id, ])

    def CommonSetPid(self, p, i, d, min, max, id=2, flags=1):
        data = [(p & 0xFFFF) >> 8, p & 0xFF, i, d, min, max, id, flags]
        self._generate(0x02, data)

    def CommonGetPidRequest(self, id=2):
        self._generate(0x03, [id, ])

    def CommonDebugTx(self, id, data):
        if (len(data) > 8):
            raise ValueError('maksymalnie 8 bajtow')
        data_ = data + [0, ]*(8-len(data)) + [id, ]
        self._generate(0x04, data_)

    def MasterSetLink(self, link):
        self._generate(0x20, [link, ])

    def MasterSetStatusMode(self, mode):
        self._generate(0x21, [mode, ])

    def MasterSetVideoChannel(self, channel):
        self._generate(0x22, [channel, ])

    def MasterComputerPowerOn(self):
        self._generate(0x23)

    def MasterComputerReset(self):
        self._generate(0x24)

    def MasterRaspberryReset(self):
        self._generate(0x25)

    def MasterSetDebugInfo(self, info):
        self._generate(0x26, [info, ])

    def MasterSetIndicator(self, state):
        self._generate(0x27, [state, ])

    def MasterSetPtzMove(self, id, y, x, zoom):
        self._generate(0x28, [id, y & 0xFF, x & 0xFF, zoom & 0xFF])

    def MotorSetWheels(self, speed, pos):
        if (len(speed) != 4):
            raise ValueError('wymagane 4 wartosci predkosci!')
        if (len(pos) != 4):
            raise ValueError('wymagane 4 wartosci pozycji!')
        data = []
        for i in speed:
            data.append(i & 0xFF)
        for i in pos:
            data.append(i & 0xFF)
        self._generate(0x40, data)
    
    def MotorSetWheels4(self, wheel0, wheel1, wheel2, wheel3):
        data = []
        pos = 0
        data.append((wheel0) & 0xFF)
        data.append((wheel1) & 0xFF)
        data.append((wheel2) & 0xFF)
        data.append((wheel3) & 0xFF)
        data.append((pos) & 0xFF)
        data.append((pos) & 0xFF)
        data.append((pos) & 0xFF)
        data.append((pos) & 0xFF)
        self._generate(0x40, data)


    def MotorSetWheels2(self, speed, pos):
        data = []
        for i in range(4):
            data.append((speed) & 0xFF)
        for i in range(4):
            data.append((pos) & 0xFF)
        self._generate(0x40, data)

    def MotorCalibrate(self, wheel_id):
        if wheel_id > 3:
            raise ValueError('niepoprawna wartosc')
        data = [ord(i) for i in 'do it'] + [wheel_id, ]
        self._generate(0x41, data)

    def ArmSetPos(self, positions):
        if (len(positions) != 6):
            raise ValueError('6 elements required')
        data = []
        for pos in positions:
            data.append((pos & 0xFFFF) >> 8)
            data.append(pos & 0xFF)
        self._generate(0x50, data)

    def ArmSetPos2(self, pos):
        data = []
        for i in range(6):
            data.append((pos & 0xFFFF) >> 8)
            data.append(pos & 0xFF)
        self._generate(0x50, data)

    def ArmSetMagnet(self, state):
        self._generate(0x51, [state, ])

    def ArmCalibrate(self, joint_id):
        if joint_id > 4:
            raise ValueError('niepoprawna wartosc')
        data = [ord(i) for i in 'do it'] + [joint_id, ]
        self._generate(0x52, data)

    def ArmGetVoltageRequest(self):
        self._generate(0x53)

    def NewArmSetVel(self, values):
        data = []
        for i in range(6):
            data.append((values[i] & 0xFFFF) >> 8)
            data.append(values[i] & 0xFF)

        data.append((values[6] & 0xFF))

        self._generate(0xE1, data)

    def NewArmSetGripper(self, value):
        data = []

        data.append((value & 0xFFFF) >> 8)
        data.append(value & 0xFF)

        self._generate(0xE2, data)

    def LidarSetSpeed(self, speed):
        data = [(speed & 0xFFFF) >> 8, speed & 0xFF]
        self._generate(0x60, data)

    def LidarSetOvercurrent(self, oc):
        self._generate(0x61, [oc, ])

    def LidarGetSpeedPosRequest(self):
        self._generate(0x62)

    def UniversalSetBridge(self, id, flags, values):
        if len(values) != 2:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append((i & 0xFFFF) >> 8)
            data.append(i & 0xFF)
        self._generate(0x70, data)

    def UniversalSetServo(self, id, flags, values):
        if len(values) != 4:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append(i & 0xFF)
        self._generate(0x71, data)

    def UniversalSetPwm(self, id, flags, values):
        if len(values) != 6:
            raise ValueError('bledna dlugosc values')
        data = [id, flags]
        for i in values:
            data.append(i & 0xFF)
        self._generate(0x72, data)

    def UniversalSetGpio(self, id, flags1, flags2, gpio1, gpio2):
        data = [id, flags1, flags2, gpio1, gpio2]
        self._generate(0x73, data)

    def UniversalGetWeightRequest(self, id):
        self._generate(0x74, [id, ])

    def ScienceGetSamplesRequest(self, id):
        self._generate(0xA0, [id, ])

    def ScienceGetWeightRequest(self, id):
        self._generate(0xA1, [id, ])

    def ScienceGetAtmosphereRequest(self, id):
        self._generate(0xA2, [id, ])

    def CustomToUart(self, args=[]):
        self._generate(0x80, args)

    def CustomToRf(self, args=[]):
        self._generate(0x81, args)

    def MuxSetCam(self, id, camera):
        self._generate(0xB1, [id, camera])

    def MuxSetChannel(self, id, channel):
        self._generate(0xB2, [id, channel])

    def MuxSetPower(self, id, power):
        self._generate(0xB3, [id, power])

    # na potrzeby testow, niespecjalnie istotne

    def pidrequest(self, s):
        s = s.replace('><', '>\n<')
        for i in s.split('\n'):
            cmd, args = self.decode(i)
            print('---')
            print('p,i,d =', (args[0] << 8)+args[1], args[2], args[3])
            print('min,max =', args[4], args[5])
            print('flags =', args[7])

    def motor(self, s):
        s = s.replace('><', '>\n<')
        for i in s.split('\n'):
            cmd, args = self.decode(i)
            speed, angle = [], []
            for i in range(4):
                speed.append((args[2*i] << 8)+args[2*i+1])
                angle.append(args[8+i])
                if (speed[i] >= (2**16/2)):
                    speed[i] -= 65536
                if (angle[i] >= (2**8/2)):
                    angle[i] -= 256
            print('speed = ', speed, '  angle =', angle)

    #########################################################
    #               generacja i dekodowanie ramek

    def handle_arm(self, args):
        converted_positions = []
        for i, item in enumerate(args):
            if not i % 2:
                converted_positions.append(args[i]*256+args[i+1])
        self.old_arm_pos = converted_positions

    def parse(self, s):  # jako argument string z pojedyncza ramka
        result = self.decode(s)
        s = ""
        if result != None:
            cmd, args = result
            if cmd == 0x5F:
                self.handle_arm(args)
            if cmd in [0x8F, 0xAD, 0xAE, 0xAF]:
                self.science.append((cmd, args))
            if cmd not in self.RX_FRAMES_FILTER:
                for i in args:
                    if self.PRINT_HEX:
                        s += "%02X " % i
                    else:
                        s += "%03i " % i
                s = ("cmd=0x%02X  n=%03i  args=" % (cmd, len(args))) + s
        return s

    def decode(self, frame):  # jako argument string z pojedyncza ramka
        try:
            arr = []
            crc = 0
            if frame[0] == self.START_BYTE:
                frame = frame[1:frame.find(self.STOP_BYTE)]
            while frame != '':
                arr.append(self.hex2byte(frame[0:2]))
                frame = frame[2:]
            for a in arr[:-1]:
                crc ^= a

            cmd = arr[0]
            if self.ARGS_COUNT_BYTES == 2:
                count = (arr[1] << 8) + arr[2]
                args = args = arr[3:-1]
            else:
                count = arr[1]
                args = args = arr[2:-1]
            _crc = arr[-1]
            if (_crc == crc) and (count == len(arr)-2 - self.ARGS_COUNT_BYTES):
                return [cmd, args]
        finally:
            pass
        return None

    # nakladka na generate

    def _generate(self, cmd, args=[]):
        frame = self.generate(cmd, args)
        if self.DISABLE_TX:
            print(frame)
        else:
            self.write(frame)

    def generate(self, cmd, args=[]):  # wszystkie argumenty to bajty
        result = self.START_BYTE + self.byte2hex(cmd)
        crc = cmd
        if self.ARGS_COUNT_BYTES == 2:
            count1 = (len(args) >> 8) & 0xFF
            count2 = (len(args)) & 0xFF
            result += self.byte2hex(count1) + self.byte2hex(count2)
            crc ^= count1 ^ count2
        else:
            count2 = (len(args)) & 0xFF
            result += self.byte2hex(count2)
            crc ^= count2

        for arg in args:
            result += self.byte2hex(arg)
            crc ^= arg
        result += self.byte2hex(crc) + self.STOP_BYTE
        return result

    def byte2hex(self, byte):
        try:
            return hex(byte)[2:].rjust(2, '0')
        except:
            return '--'

    def hex2byte(self, h):
        try:
            return int(h, 16)
        except:
            return 0

    #########################################################
    #       warstwa niskopoziomowa - obsluga transmisji
    port = serial.Serial()
    port.timeout = 0

    last_rx_time = time.time()

    logfile = None
    rxbuf = ""
    lock = threading.Lock()  # na potrzeby wielowatkowego printa

    def search_rx_buf(self):
        if self.DISABLE_DECODING:
            if (self.rxbuf.find('\n') > -1):
                self.rxbuf = self.rxbuf.replace('\r\n', '\n')
                logfile.write(self.rxbuf[:-1])
                self.lock.acquire()
                print(self.rxbuf)
                self.lock.release()
                self.rxbuf = ""
        else:
            pos = self.rxbuf.find(self.STOP_BYTE)
            if (pos > -1):
                raw_frame = self.rxbuf[:pos]
                self.rxbuf = self.rxbuf[pos+1:]

                pos = raw_frame.rfind(self.START_BYTE)
                if (pos > -1):
                    raw_frame = raw_frame[pos+1:]
                    s = self.parse(raw_frame)
                    if s != "":
                        s = "[RX] %s\n" % s
                    elif not self.DISABLE_RAW_RX:
                        s = "[RX][RAW] %s\n" % raw_frame
                    logfile.write(s)
                    if self.PRINT_RX:
                        self.lock.acquire()
                        print(s[:-1])
                        self.lock.release()

                # zapetlenie do momentu sparsowania wszystkich pelnych ramek
                if len(self.rxbuf) > 0:
                    self.search_rx_buf()

    def rxhandler(self, serialport):
        global rx_running, rxbuf
        rx_running = True
        while rx_running:
            count = serialport.inWaiting()
            if (count > 0):
                self.last_rx_time = time.time()
                self.rxbuf += str(serialport.read(count))
                self.search_rx_buf()

    def write(self, data):
        global logfile
        if not self.port.isOpen():
            self.start()
        self.port.write(data.encode())

        s = self.parse(data)
        if s != "":
            s = "[TX] %s\n" % s
        else:
            s = "[TX][RAW] %s\n" % data
        logfile.write(s)
        if self.PRINT_TX:
            self.lock.acquire()
            print(s[:-1])
            self.lock.release()

    def start(self):
        global logfile, BAUDRATE, port

        self.port.port = self.PORTNAME
        self.port.baudrate = self.BAUDRATE
        logfile = open(time.strftime("latest.log"), 'a')
        if not self.port.isOpen():
            self.port.open()

        thread = threading.Thread(target=self.rxhandler, args=(self.port,))
        thread.start()

    def stop(self):
        global rx_running, logfile
        rx_running = False
        self.port.close()
        logfile.close()

    def extend_rx_filter(ids):
        filters_to_add = [i for i in ids if i not in Uart.RX_FRAMES_FILTER]
        new_filters = [i for i in Uart.RX_FRAMES_FILTER] + filters_to_add
        Uart.RX_FRAMES_FILTER = tuple(new_filters)
