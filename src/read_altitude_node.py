import serial
import time
from threading import Thread
class read_altitude(Thread):
    #This class is going to feedback the altitude captured by tof sensor
    def __init__(self):
        #create pipe
        #super(read_altitude(), self).__init__()
        #create serial read
        Thread.__init__(self)
        self.ser = serial.Serial("/dev/serial0", 115200,timeout=0) # mini UART serial device
        
    def getTFminiData(self):
        while True:
            count = self.ser.in_waiting
            if count > 8:
                recv = self.ser.read(9)
                self.ser.reset_input_buffer()
                if recv[0] == 0x59 and recv[1] == 0x59:
                    distance = recv[2] + recv[3] * 256
                    distance_meter = distance/100
                    print('The distance is: ',distance_meter)
                    self.ser.reset_input_buffer()
                #self.pipe_write.recv()     
    def run(self):
        try:
            if self.ser.is_open == False:
                self.ser.open()
            self.getTFminiData()
        except KeyboardInterrupt:
            if self.ser != None:
                self.ser.close()
if __name__ == '__main__':
    try:
        altitude_thread = read_altitude()
        altitude_thread.start()
    except:
        print("error")






        