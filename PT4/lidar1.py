import serial
import time

ser = serial.Serial("/dev/ttyTHS1", 115200)

while True:
        #time.sleep(0.1)
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)  
            ser.reset_input_buffer()  
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance_l = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                print( distance_l)
                ser.reset_input_buffer()
                




