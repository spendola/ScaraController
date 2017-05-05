import serial
import time
import numpy as np
from array import array
import cv2

class SerialComm(object):
    
    def __init__(self, port, timeout=5):
        self.arduino = None
        attempt = 0
        while self.arduino == None and attempt < timeout:
            self.arduino = self.OpenConnection(port)
            attempt = attempt + 1

        if(self.arduino != None):
            print("waiting for handshake")
            self.Handshake("Ready")
            print("handshake received")

            print("waiting for arm to reference itself")
            self.Handshake("Referenced")
            print("Arm is Referenced")
            
    def OpenConnection(self, portAddress):
        portAddress = portAddress if portAddress != "" else "/dev/tty.usbserial"
        print("Connecting to " + portAddress)
        try:
            ser = serial.Serial(portAddress, 9600, timeout = 1)
            return ser
        except Exception, e:
            print "error open serial port: " + str(e)
        return None

    def Handshake(self, expected):
        try:
            print("Handshake for " + expected)
            response = self.Read()
            while response != expected:
                print "-> '" + response + "'"
                response = self.Read()
                time.sleep(0.25)
            print "-> '" + response + "'"
            self.arduino.write('k')
            time.sleep(0.25)
        except:
            return "e"

    def Read(self):
        try:
            return self.arduino.readline().replace("\n", "")[:-1]
        except:
            return "e"
        
    def Write(self, outByte):
        try:
            return self.arduino.write(outByte)
        except:
            return "e"

    def WritePosition(self, x, y):
        try:
            print("Sending Position: " + str(x) + ", " + str(y)) 
            msg = bytearray([x+64, y+64, 'm'])
            bytesWriten = self.arduino.write(msg)
            print(str(bytesWriten) + " bytes sent")
            time.sleep(0.25)
            self.Handshake("m");
        except:
            return "e"

##def main():
##    print("Serial to Arduino")
##    # port = raw_input("Enter the port address: ")
##    
##    arduino = None
##    timeout = 0
##    while arduino == None and timeout < 10:
##        arduino = OpenConnection("/dev/cu.usbmodem1411")
##        timeout = timeout + 1
##
##    if(arduino != None):
##        print("waiting for handshake")
##        Handshake(arduino, "Ready")
##        print("handshake received")
##
##        print("waiting for arm to reference itself")
##        Handshake(arduino, "Referenced")
##        print("Arm is Referenced")
##
##        time.sleep(1)
##        while(True):
##            x, y = GetDestination()
##            msg = bytearray([x, y, 'm'])
##            bytesWriten = arduino.write(msg)
##            print msg
##            print("sending " + str(msg) + " (" + str(bytesWriten) + " bytes)")
##            time.sleep(0.25)
##            Handshake(arduino, "m");
##
##    camera = cv2.VideoCapture(0)
##    camera.set(3, 320)
##    camera.set(4, 240)
##    while(True):
##        ret, frame = camera.read()
##        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
##        FindCenterOfMass(frame, 320, 240)
##        cv2.imshow('Video', frame)
##        if(cv2.waitKey(1) & 0XFF == ord('q')):
##            print frame.shape
##            break
##    camera.release()
##    cv2.destroyAllWindows()
##
##def FindCenterOfMass(frame, width, height):
##    cols = np.zeros(width)
##    mass, center = 0, 0
##    # calculate total visible mass
##    for c in range(width):
##        temp = 0
##        for r in range(height):
##            temp = temp + frame[r][c][0]
##        cols[c] = temp
##        
##    mass = np.sum(cols)
##    center = mass/2
##    
##    # find center of visible mass
##    for c in range(width):
##        center = center - cols[c]
##        if(center <= 0):
##            print("Center: " + str(c))
##            break
##
##def Handshake(arduino, expected):
##    print("Handshake for " + expected)
##    response = ReadSerial(arduino)
##    while response != expected:
##        print "-> '" + response + "'"
##        response = ReadSerial(arduino)
##        time.sleep(0.25)
##    print "-> '" + response + "'"
##    arduino.write('k')
##    time.sleep(0.25)
##    #arduino.flushInput()
##    
##
##def ReadSerial(arduino):
##    try:
##        return arduino.readline().replace("\n", "")[:-1]
##    except:
##        return "e"
##        
##def GetResponse(arduino):
##    response = ""
##    while response == "":
##        response = ar
##        duino.readline()
##        arduino.flushOutput()
##        time.sleep(0.25)
##    return response
##                 
##def GetDestination():
##    print("Destination:")
##    try:
##        x = int(raw_input("X: "))
##        y = int(raw_input("Y: "))
##        return x, y
##    except:
##        print("Terminating")
##        return None, None
##    
##def OpenConnection(portAddress):
##    portAddress = portAddress if portAddress != "" else "/dev/tty.usbserial"
##    print("Connecting to " + portAddress)
##    try:
##        ser = serial.Serial(portAddress, 9600, timeout = 1)
##        return ser
##    except Exception, e:
##        print "error open serial port: " + str(e)
##    return None
##
##if __name__ == '__main__':
##    main()
