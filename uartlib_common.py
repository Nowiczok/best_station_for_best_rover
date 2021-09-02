import serial, time, threading, struct

#########################################################
##                       UARTlib v2
##                 Patryk Jarosz, 08.2019
##
## Ponizszy skrypt dostarcza funkcje do obslugi ramek zgodnych z 
## protokolem UARTlib oraz obsluguje ich nadawanie i odbior przez 
## zadany port szeregowy. Odbior zrealizowany jest na osobnym watku.
## Dodatkowo tworzony jest plik z logami (nadawanie i odbior).
##
## Uzytkowanie:
##    start() - otwieranie portu, uruchamianie watku odbiorczego
##    stop() - zamykanie portu, wylaczanie watku odbiorczego
    
    
#########################################################
#                   konfiguracja

PORTNAME = 'COM3'
BAUDRATE = 115200
ARGS_COUNT_BYTES = 1 #dlugosc pola ilosci argumentow (1 lub 2 bajty)
DISABLE_TX = False #wylaczanie wysylania danych (samo generowanie i wyswietlanie)
PRINT_TX = False #rownolegle wyswietlanie nadawanych ramek
PRINT_RX = True #rownolegle wyswietlanie odebranych ramek
DISABLE_DECODING = False #wylaczenie dekodowania ramek odbiorczych, wyswietlane surowe dane przychodzace z urzadzenia
PRINT_HEX = True #wyswietlanie argumentow szesnastkowo
DISABLE_RAW_RX = True

START_BYTE = '<' #nie zmieniac
STOP_BYTE = '>' #nie zmieniac

RX_FRAMES_FILTER = (0x5F,)
#########################################################
#      warstwa komunikacyjna - ramki na magistrali

def CanCommonResetAll():
    data = [ord(i) for i in 'do it']
    _generate(0x30, data)

def CanCommonResetDevice(id):
    data = [ord(i) for i in 'do it']
    data.append(id)
    _generate(0x31, data)

def CanCommonSetPid(p, i, d, min, max, id=2, flags=1):
    data = [(p&0xFFFF)>>8, p&0xFF, i, d, min, max, id, flags]
    _generate(0x32, data)

def CanCommonGetPidRequest(id=2):
    data = [id,]
    _generate(0x33, data)


def CanArmSetPos(positions):
    if (len(positions) != 6): raise ValueError('6 elements required')
    data = []
    for pos in positions:
        data.append((pos&0xFFFF)>>8)
        data.append(pos&0xFF)
    _generate(0x20, data[:8]) + generate(0x21, data[8:12])

def CanArmSetPos3(positions):
    if (len(positions) != 6): raise ValueError('6 elements required')
    data = []
    for pos in positions:
        data.append((pos&0xFFFF)>>8)
        data.append(pos&0xFF)
    _generate(0x21, data[8:12])

def CanArmSetPos2(pos):
    data = []
    for i in range(6):
        data.append((pos&0xFFFF)>>8)
        data.append(pos&0xFF)
    _generate(0x20, data[:8]) + generate(0x21, data[8:12])

def CanArmSetMagnet(state = True):
    data = [0,]
    if (state): data[0] = 1
    _generate(0x22, data)

def CanArmCalibrate(joint):
    data = [ord(i) for i in 'do it']
    data.append(joint)
    _generate(0x23, data)

def CanArmDebugTx(data):
    data = data + [0,]*(8 - len(data))
    _generate(0x24, data)

def CanArmDebugTxInfo(flags):
    ArmDebugTx([1 , flags,])

def CanArmDebugTxProtection(uvpen, uvpdis, ocp):
    data = [2, (uvpen&0xFFFF)>>8, uvpen&0xFF, (uvpdis&0xFFFF)>>8, uvpdis&0xFF, ocp]
    ArmDebugTx(data)

def CanArmDebugTxGripper(raw):
    data = [3, (raw&0xFFFF)>>8, raw&0xFF] + [ord(i) for i in 'do it']
    ArmDebugTx(data)

def CanArmPoll():
    _generate(0x27)


def CanMotorSetWheels(speed, pos):
    if (len(speed) != 4): raise ValueError('wymagane 4 wartosci predkosci!')
    if (len(pos) != 4): raise ValueError('wymagane 4 wartosci pozycji!')
    
    data = []
    for i in speed:
        data.append(i&0xFF)
    for i in pos:
        data.append(i&0xFF)
    _generate(0x10, data)

def CanMotorSetWheels2(speed, pos):
    data = []
    for i in range(4):
        data.append((speed)&0xFF)
    for i in range(4):
        data.append((pos)&0xFF)
    _generate(0x10, data)

def CanMotorCalibrate(channel):
    data = [ord(i) for i in 'do it'] + [channel,]
    _generate(0x11, data)   
    
def CanMotorDebugTx(data):
    data = data + [0,]*(8 - len(data))
    _generate(0x12, data)

def CanMotorDebugTxInfo(flags):
    CanMotorDebugTx([1, flags,])

def CanMotorDebugTxProtection(uvpen, uvpdis, ocp):
    data = [(uvpen&0xFFFF)>>8, uvpen&0xFF, (uvpdis&0xFFFF)>>8, uvpdis&0xFF, ocp]
    CanMotorDebugTx([2,] + data)

def CanMotorDebugTxSpeed(max_speed):
    CanMotorDebugTx([3, max_speed])

def CanMotorPoll():
    _generate(0x17)


def CanPartySetIgniters(flags):
    _generate(0x50, [flags,])

def CanPartySetAuto(flags):
    _generate(0x51, [flags,])

def CanPartySetRgb(anim_type):
    data = [anim_type,] + [0,]*5
    _generate(0x52, data)

def CanPartySetPower(flags):
    _generate(0x53, [flags,])

def CanPartyDebugTx(data):
    _generate(0x54, data)


def CanUniversalSetBridge(id, flags, values):
    if len(values) != 2: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append((i&0xFFFF)>>8)
        data.append(i&0xFF)
    _generate(0x80, data)

def CanUniversalSetServo(id, flags, values):
    if len(values) != 4: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append(i&0xFF)
    _generate(0x81, data)

def CanUniversalSetPwm(id, flags, values):
    if len(values) != 6: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append(i&0xFF)
    _generate(0x82, data)

def CanUniversalDebugTx(data):
    data = data + [0,]*(8 - len(data))
    _generate(0x84, data)

def CanUniversalDebugTxInfo(flags):
    CanUniversalDebugTx([1 , flags,])

def CanUniversalDebugTxProtection(uvpen, uvpdis):
    data = [2, (uvpen&0xFFFF)>>8, uvpen&0xFF, (uvpdis&0xFFFF)>>8, uvpdis&0xFF]
    CanUniversalDebugTx(data)

def CanUniversalDebugTxDefaultServo(flags, values):
    if len(values) != 4: raise ValueError('wymagane 4 wartosci pozycji!')
    for i in range(4):
        if (values[i]) > 100: values[i] = 100
        if (values[i]) < 0: values[i] = 0
    CanUniversalDebugTx([3, flags,] + values)


def CanSciencePoll(id):
   _generate(0x90, [id,])

def CanScienceDebugTx(data):
    data = data + [0,]*(8 - len(data))
    _generate(0x91, data)

def CanScienceDebugTxInfo(flags):
    CanScienceDebugTx([1 , flags,])

def MkArmJointPositioningStart(jointId):
    data = [jointId]
    print data
    _generate(0xBB, data)

def MkArmJointPositioningAbort(jointId):
    data = [jointId]
    print data
    _generate(0xBC, data)

def MkArmTorque(jointId,torq):
    data = [jointId]
    x = list(struct.pack("<f", torq))
    for i in x:
        data.append(ord(i))
    print data
    _generate(0xBF, data)
    
def MkArmVel(jointId,vel, acc, torq):
    data = [jointId]
    x = list(struct.pack("<fff", torq, acc, vel))
    for i in x:
        data.append(ord(i))
    print data
    _generate(0xC0, data)
    

def MkArmPos(jointId, pos, vel, acc, torq):
    data = [jointId]
    x = list(struct.pack("<ffff", torq, acc, vel, pos))
    for i in x:
        data.append(ord(i))
    print data
    _generate(0xC1, data)





#########################################################
# warstwa komunikacyjna - ramki protokolu 433/WiFi/Auto

def CommonResetAll():
    _generate(0x00)

def CommonResetDevice(id):
    _generate(0x01, [id,])

def CommonSetPid(p, i, d, min, max, id=2, flags=1):
    data = [(p&0xFFFF)>>8, p&0xFF, i, d, min, max, id, flags]
    _generate(0x02, data)

def CommonGetPidRequest(id=2):
    _generate(0x03, [id,])

def CommonDebugTx(id, data):
    if (len(data) > 8): raise ValueError('maksymalnie 8 bajtow')
    data_ = data + [0,]*(8-len(data)) + [id,]
    _generate(0x04, data_)

def MasterSetLink(link):
    _generate(0x20, [link,])

def MasterSetStatusMode(mode):
    _generate(0x21, [mode,])

def MasterSetVideoChannel(channel):
    _generate(0x22, [channel,])

def MasterComputerPowerOn():
    _generate(0x23)

def MasterComputerReset():
    _generate(0x24)
    
def MasterRaspberryReset():
    _generate(0x25)
    
def MasterSetDebugInfo(info):
    _generate(0x26, [info,])

def MasterSetIndicator(state):
    _generate(0x27, [state,])

def MasterSetPtzMove(id, y, x, zoom):
    _generate(0x28, [id, y&0xFF, x&0xFF, zoom&0xFF])

def MotorSetWheels(speed, pos):
    if (len(speed) != 4): raise ValueError('wymagane 4 wartosci predkosci!')
    if (len(pos) != 4): raise ValueError('wymagane 4 wartosci pozycji!')
    data = []
    for i in speed:
        data.append(i&0xFF)
    for i in pos:
        data.append(i&0xFF)
    _generate(0x40, data)

def MotorSetWheels2(speed, pos):
    data = []
    for i in range(4):
        data.append((speed)&0xFF)
    for i in range(4):
        data.append((pos)&0xFF)
    _generate(0x40, data)

def MotorCalibrate(wheel_id):
    if wheel_id > 3: raise ValueError('niepoprawna wartosc')
    data = [ord(i) for i in 'do it'] + [wheel_id,]
    _generate(0x41, data)

def ArmSetPos(positions):
    if (len(positions) != 6): raise ValueError('6 elements required')
    data = []
    for pos in positions:
        data.append((pos&0xFFFF)>>8)
        data.append(pos&0xFF)
    _generate(0x50, data)

def ArmSetPos2(pos):
    data = []
    for i in range(6):
        data.append((pos&0xFFFF)>>8)
        data.append(pos&0xFF)
    _generate(0x50, data)

def ArmSetMagnet(state):
    _generate(0x51, [state,])

def ArmCalibrate(joint_id):
    if joint_id > 4: raise ValueError('niepoprawna wartosc')
    data = [ord(i) for i in 'do it'] + [joint_id,]
    _generate(0x52, data)

def ArmGetVoltageRequest():
    _generate(0x53)

def LidarSetSpeed(speed):
    data = [(speed&0xFFFF)>>8, speed&0xFF]
    _generate(0x60, data)

def LidarSetOvercurrent(oc):
    _generate(0x61, [oc,])

def LidarGetSpeedPosRequest():
    _generate(0x62)

def UniversalSetBridge(id, flags, values):
    if len(values) != 2: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append((i&0xFFFF)>>8)
        data.append(i&0xFF)
    _generate(0x70, data)

def UniversalSetServo(id, flags, values):
    if len(values) != 4: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append(i&0xFF)
    _generate(0x71, data)

def UniversalSetPwm(id, flags, values):
    if len(values) != 6: raise ValueError('bledna dlugosc values')
    data = [id, flags]
    for i in values:
        data.append(i&0xFF)
    _generate(0x72, data)

def UniversalSetGpio(id, flags1, flags2, gpio1, gpio2):
    data = [id, flags1, flags2, gpio1, gpio2]
    _generate(0x73, data)

def UniversalGetWeightRequest(id):
    _generate(0x74, [id,])

def ScienceGetSamplesRequest(id):
    _generate(0xA0, [id,])

def ScienceGetWeightRequest(id):
    _generate(0xA1, [id,])

def ScienceGetAtmosphereRequest(id):
    _generate(0xA2, [id,])

    

#na potrzeby testow, niespecjalnie istotne
def pidrequest(s):
    s = s.replace('><', '>\n<')
    for i in s.split('\n'):
        cmd, args = decode(i)
        print '---'
        print 'p,i,d =', (args[0]<<8)+args[1], args[2], args[3]
        print 'min,max =', args[4], args[5]
        print 'flags =', args[7]

def motor(s):
    s = s.replace('><', '>\n<')
    for i in s.split('\n'):
        cmd, args = decode(i)
        speed, angle = [],[]
        for i in range(4):
            speed.append((args[2*i]<<8)+args[2*i+1])
            angle.append(args[8+i])
            if (speed[i] >= (2**16/2)): speed[i] -= 65536
            if (angle[i] >= (2**8/2)): angle[i] -= 256
        print 'speed = ', speed, '  angle =', angle

    
    
#########################################################
#               generacja i dekodowanie ramek

def parse(s): #jako argument string z pojedyncza ramka 
    result = decode(s)
    s = ""
    if result != None:
        cmd, args = result

        if cmd not in RX_FRAMES_FILTER:
          for i in args:
              if PRINT_HEX: s += "%02X " % i
              else: s += "%03i " % i
          s = ("cmd=0x%02X  n=%03i  args=" % (cmd, len(args))) + s
    return s

def decode(frame): #jako argument string z pojedyncza ramka 
    try:
        arr = []
        crc = 0
        if frame[0] == START_BYTE:
            frame = frame[1:frame.find(STOP_BYTE)]
        while frame != '':
            arr.append(hex2byte(frame[0:2]))
            frame = frame[2:]
        for a in arr[:-1]:
            crc ^= a

        cmd = arr[0]
        if ARGS_COUNT_BYTES == 2:
            count = (arr[1]<<8) + arr[2]
            args =  args = arr[3:-1]
        else:
            count = arr[1]
            args =  args = arr[2:-1]
        _crc = arr[-1]
        if (_crc == crc) and (count == len(arr)-2-ARGS_COUNT_BYTES):
            return [cmd, args]
    finally:
        pass
    return None

#nakladka na generate
def _generate(cmd, args=[]):
    frame = generate(cmd, args)
    if DISABLE_TX: print frame
    else: write(frame)
    
def generate(cmd, args=[]): #wszystkie argumenty to bajty
    result = START_BYTE + byte2hex(cmd) 
    crc = cmd
    if ARGS_COUNT_BYTES == 2:
        count1 = (len(args) >> 8) & 0xFF
        count2 = (len(args)     ) & 0xFF
        result += byte2hex(count1) + byte2hex(count2)
        crc ^= count1 ^ count2
    else:
        count2 = (len(args)     ) & 0xFF
        result += byte2hex(count2)    
        crc ^= count2
        
    for arg in args:
        result += byte2hex(arg)
        crc ^= arg
    result += byte2hex(crc) + STOP_BYTE
    return result

def byte2hex(byte):
    try:
        return hex(byte)[2:].rjust(2,'0')
    except:
        return '--'

def hex2byte(h):
    try:
        return int(h, 16)
    except:
        return 0




#########################################################
#       warstwa niskopoziomowa - obsluga transmisji

port = serial.Serial()
port.port = PORTNAME
port.baudrate = BAUDRATE
port.timeout = 0

logfile = None
rxbuf = ""
lock = threading.Lock() #na potrzeby wielowatkowego printa

def search_rx_buf():
    global rxbuf, logfile
    if DISABLE_DECODING:
        if (rxbuf.find('\n') > -1):
            rxbuf = rxbuf.replace('\r\n', '\n')
            logfile.write(rxbuf[:-1])
            lock.acquire()
            print rxbuf
            lock.release()
            rxbuf = ""
    else:        
        pos = rxbuf.find(STOP_BYTE)
        if (pos > -1):
            raw_frame = rxbuf[:pos]
            rxbuf = rxbuf[pos+1:]

            pos = raw_frame.rfind(START_BYTE)
            if (pos > -1):
                raw_frame = raw_frame[pos+1:]
                s = parse(raw_frame)
                if s != "":
                    s = "[RX] %s\n" % s
                elif not DISABLE_RAW_RX:
                    s = "[RX][RAW] %s\n" % raw_frame
                logfile.write(s)
                if PRINT_RX:
                    lock.acquire()
                    print s[:-1]
                    lock.release()

            #zapetlenie do momentu sparsowania wszystkich pelnych ramek
            if len(rxbuf) > 0:
                search_rx_buf()


def rxhandler(serialport):
    global rx_running, rxbuf
    rx_running = True
    while rx_running:
        count = serialport.inWaiting()
        if (count>0):
            rxbuf += serialport.read(count)
            search_rx_buf()
            

def write(data):
    global logfile
    if not port.isOpen():
        start()
    port.write(data)

    s = parse(data)
    if s != "":
        s = "[TX] %s\n" % s
    else:
        s = "[TX][RAW] %s\n" % data
    logfile.write(s)
    if PRINT_TX:
        lock.acquire()
        print s[:-1]
        lock.release()
    

def start():
    global logfile, BAUDRATE, port
    port.baudrate = BAUDRATE
    logfile = open(time.strftime("%Y%m%d %H%M%S.txt"), 'a')
    if not port.isOpen():
        port.open()

    thread = threading.Thread(target=rxhandler, args=(port,))
    thread.start()


def stop():
    global rx_running, logfile
    rx_running = False
    port.close()
    logfile.close()
