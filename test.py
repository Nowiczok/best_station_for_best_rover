import serial

# msg.data = '\x01\x08\x20\x00\x00\x00\x00\x00\x00\x00'
class SerialWrapper:
  def __init__(self, device):
    self.ser = serial.Serial(device, 38400)

  def drive(self, pp, pt, lt, lp):
    data = "<408" + hex(pp)[2:] + hex(pt)[2:] + hex(lt)[2:] + hex(lp)[2:] + "00000000>"
    self.ser.write(data.encode())

  def turn(self, pp, pt, lt, lp):
    data = "<40800000000" + hex(pp)[2:] + hex(pt)[2:] + hex(lt)[2:] + hex(lp)[2:] + ">"
    self.ser.write(data.encode())

def main():
  ser = SerialWrapper('COM6')
  while 1:
    #misc code here
    # ser.sendData(data)
    break