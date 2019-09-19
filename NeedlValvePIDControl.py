# This program is to communicate with a pressure gauge and an Arduino to control a stepper motor, in order to rotate a needle valve.
# needleValveStepper.ino should be successfully uploaded to Arduino before run this program

import time
import serial

class PID:
    def __init__(self, P=5, I=0.05, D=0.0):  #set the values of P, I, D
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        #set maxoutput, the number of steps the motor can move without time interval(time.sleep())
        self.maxoutput = 10.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
        
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        #set windup_guard, the maximum of ITerm
        self.windup_guard = 10.0
        self.output = 0.0
        
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        print('delta_time: ',delta_time)
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error    
            print('PTerm: ',self.PTerm)
            self.ITerm += error * delta_time
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            print('self.ITerm: ',self.ITerm)
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            print('self.DTerm: ',self.DTerm)
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.output > self.maxoutput:
                self.output = self.maxoutput
            if self.output < -self.maxoutput:
                self.output = -self.maxoutput
        return self.output
        
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
        
    def setKi(self, integral_gain):
        self.Ki = integral_gain
        
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
        
    def setWindup(self, windup):
        self.windup_guard = windup
        
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
        
    def setMaxOutput(self, maxoutput):
        self.maxoutput = maxoutput

if __name__ == "__main__":
    pid=PID()
    # default the list read from pressure gauge
    PIDinput = ['0', '0', '0', '0', '0', '0', '0', '0', '0']  
    # default the string which will be sent to arduino
    PIDoutput = ''  
    # set desired pressure, unit: torr
    pid.SetPoint = 3*pow(10,0)    
    feedback_value = 0
    output_value = 0
    #open serial for arduino
    ser_arduino = serial.Serial('COM7', baudrate = 9600, timeout=2)  
    #open serial for pressure gauge
    ser_pressure = serial.Serial('COM10', baudrate = 9600, stopbits = 1, parity = 'N', timeout = 2)  
    while True:
        print('start')
        # send command to pressure gauge to ask pressure
        ser_pressure.write('RD\r')    
        print('send cmd to pressure gauge')
        for n in range(0, 9):
            PIDinput[n] = ser_pressure.read(1)
        print('read from pressure gauge:',PIDinput)  #print the pressure sent back by pressure gauge, unit: torr
        
        #convert the pressure to float
        if PIDinput[5]== '+':
            feedback_value = (int(PIDinput[0])+0.1*int(PIDinput[2])+0.01*int(PIDinput[3]))*pow(10,10*int(PIDinput[6])+int(PIDinput[7]))
        elif PIDinput[5]== '-':
            feedback_value = (int(PIDinput[0])+0.1*int(PIDinput[2])+0.01*int(PIDinput[3]))*pow(10,-10*int(PIDinput[6])-int(PIDinput[7]))
        print('feedback_valueL:',feedback_value)
        
        #get the number of steps the motor will move
        output_value = pid.update(feedback_value)
        print('output_value:',output_value) 
        
        #send command to arduino to rotate the needle valve
        #tighten the needle valve
        if output_value > 0:
            for n in range(0, int(round(output_value))):
                try:
                    ser_arduino.write('2')
                except Exception:
                    ser_arduino.close()
                    ser_arduino.open()
                    ser_arduino.isOpen
            ser_arduino.write('0') #turn off the LED 
        #loosen the needle valve
        elif output_value < 0:
            for n in range(0, int(round(abs(output_value)))):
                try:
                    ser_arduino.write('1')
                except Exception:
                    ser_arduino.close()
                    ser_arduino.open()
                    ser_arduino.isOpen
            ser_arduino.write('0') #turn off the LED
        elif output_value == 0:
            try:
                ser_arduino.write('0')
            except Exception:
                ser_arduino.close()
                ser_arduino.open()
                ser_arduino.isOpen
        print('end')
        #set time interval
        time.sleep(0.1)
        
        
