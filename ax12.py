from time import sleep
from serial import Serial
import RPi.GPIO as GPIO
import sys
import time


class Ax12:
    '''
        Ax12 Servo's class
        This class allows you to control the settings and move the Ax12 servo's
        Written for the Raspberry Pi 3 and for python3 (Python2 does not work)
        This class is not thread safe, and should only be used from the main thread
    '''
    PORT = "/dev/ttyS0"
    BAUDRATE = 1000000
    TIME_OUT= 0.01

    AX_GOAL_LENGTH = 5
    AX_POS_LENGTH = 4
    #AX_VOLT_LENGTH = 4
    AX_AL_LENGTH = 7

    AX_PRESENT_VOLTAGE = 42

    AX_BYTE_READ = 1

    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_GOAL_POSITION_L = 30
    AX_PRESENT_POSITION_L = 38
    AX_CW_ANGLE_LIMIT_L = 6
    AX_START = 255
    AX_REG_WRITE = 4
    AX_INT_READ = 2
    TX_DELAY_TIME =0.0002
    #raspberry pi constants
    RPI_DIRECTION_PIN = 18
    RPI_DIRECTION_TX = GPIO.HIGH
    RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001

    #static variables
    port = None
    gpioSet = False

    class timeoutError(Exception) : pass

    def __init__(self):
        if(Ax12.port == None):
            Ax12.port =  Serial(  port=Ax12.PORT,baudrate=Ax12.BAUDRATE,timeout=Ax12.TIME_OUT)
            # Ax12.port.flush()
        if(not Ax12.gpioSet):
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(Ax12.RPI_DIRECTION_PIN, GPIO.OUT)
            Ax12.gpioSet = True
        Ax12.port.flush()
        self.direction(Ax12.RPI_DIRECTION_RX)
   
    def direction(self,d):
        '''
            Sets the pin and sleeps
            @param d, the pin you wish to set
        '''
        GPIO.output(Ax12.RPI_DIRECTION_PIN,d)
        sleep(Ax12.RPI_DIRECTION_SWITCH_DELAY)

    def readData(self,d):
        '''
            Reads the data
            @param  d, reserved parameter
            @return a number
            @except time out exception
        '''
        self.direction(Ax12.RPI_DIRECTION_RX)
        bytesToRead = 5 #Ax12.port.inWaiting()
        reply = Ax12.port.read(bytesToRead)

        try:
            assert reply[0] == 0xFF
        except:
            e = "servo : " + str(d) + " timed out"
            raise Ax12.timeoutError(e)
        try:
            length = reply[3] - 2
            error = reply[4]
            if(error != 0):
                return -error
            elif(length == 0):
                return error
            else:
                if(length > 1):
                    reply = Ax12.port.read(2)
                    return (reply[1] << 8) + (reply[0] << 0)
                else:
                    reply = Ax12.port.read(1)
                    return reply[0]
        finally:
             Ax12.port.flushInput()
             time.sleep(0.002)


    def setAngleLimit(self, id, cwLimit, ccwLimit):
        '''
            Set's the limit of an angle
            @param id   the id of the servo
            @param CWLimit  the new limit of clockwise
            @param CCWlimit the new counter clockwise
            @return bool    did it succeed?
            @except time out exception
        '''
        outData = bytearray([
            Ax12.AX_START,Ax12.AX_START, id,
            Ax12.AX_AL_LENGTH, Ax12.AX_WRITE_DATA,
            Ax12.AX_CW_ANGLE_LIMIT_L, cwLimit&0xff, cwLimit >> 8, ccwLimit&0xff, ccwLimit>>8])

        checksum = ~sum(outData[2:])&0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    '''
        pings an servo
        @param id   The servo id you wish to ping
        @return bool    did the ping succeed?
        @except Time out exception
    '''
    def ping(self,id):
        PING = 1
        outData = bytearray([Ax12.AX_START, Ax12.AX_START, id, Ax12.AX_READ_DATA, PING])
        checksum = ~sum(outData[2:]) & 0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    '''
        Checks the servo's and returns a list of available servo's
        @param minValue the starting value you wish to learn
        @param maxValue the endvalue you wish to learn
        @param verbose  Do you wish to print the status of servo's?
        @return list of id's of available servo's
    '''
    def learnServos(self,minValue, maxValue, printCallBack=lambda x: x):
        servoList = []
        for i in range(minValue, maxValue + 1):
            try:
                temp = self.ping(i)
                servoList.append(i)
                printCallBack("Found servo #" + str(i))
                time.sleep(0.1)
            except Exception as detail:
                printCallBack("Error pinging servo #" + str(i) + '; ' + str(detail))
                pass
        return servoList
  
    def readPosition(self, id):
        '''
            Reads the position of the specified servo
            @Param id the id of the servo
            @return the position of specified servo
            @except Time out exception
        '''
        outData = bytearray([
            Ax12.AX_START, Ax12.AX_START, id,
            Ax12.AX_POS_LENGTH, Ax12.AX_READ_DATA, Ax12.AX_PRESENT_POSITION_L,
            Ax12.AX_INT_READ])
        checksum = ~sum(outData[2:]) & 0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)


    def readTemperature(self,id):
        '''
            Reads the temperature from the specified servo
            @Param id  the id of the servo
            @return returns the temperature
            @except An time out exception
        '''
        TEMP_LENGTH = 4
        PRESENT_TEMPERATURE = 43
        outData = bytearray([
        Ax12.AX_START,Ax12.AX_START,id, TEMP_LENGTH,Ax12.AX_READ_DATA, PRESENT_TEMPERATURE, Ax12.AX_BYTE_READ])
        checksum = ~sum(outData[2:]) & 0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def moveSpeed(self, id, position, speed):
        '''
            Moves the specified servo with the specified speed
            @param id                   the servo id
            @param position             The position to move to
            @param speed                The movement speed
            @return succesfull?         if the servo was succesfull
            @IO exception               Timeout exception
        '''

        Ax12.port.flushInput()
        AX_GOAL_SP_LENGTH = 7
        AX_GOAL_POSITION_L = 30
        outData = bytearray([Ax12.AX_START, Ax12.AX_START, id,
                            AX_GOAL_SP_LENGTH, Ax12.AX_WRITE_DATA,
                            AX_GOAL_POSITION_L, position&0xff,
                            position>>8, speed&0xff,speed>>8])
        checksum = ~sum(outData[2:])&0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)

        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)


    def setShutdownAlarm(self,id,alarm):
        Ax12.port.flushInput()
        SHUTDOWN_ALARM_LENGTH = 4
        SHUTDOWN_ALARM = 18
        
        outData = bytearray([
            Ax12.AX_START, Ax12.AX_START, id,
            SHUTDOWN_ALARM_LENGTH, Ax12.AX_WRITE_DATA,
            SHUTDOWN_ALARM, alarm
        ])        
        checksum = ~sum(outData[2:])&0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)


    def readVoltage(self, id):
        '''
            Reads the voltage of the specified servo
            @param id               The servo 
            @return voltage         The voltage it returns
            @IO exception           An time out exception when a time-out occurs
        '''
        Ax12.port.flushInput()
        VOLT_LENGTH = 4
        outData = bytearray([
            Ax12.AX_START, Ax12.AX_START, id,
            VOLT_LENGTH, Ax12.AX_READ_DATA,
            Ax12.AX_PRESENT_VOLTAGE, Ax12.AX_BYTE_READ])

        checksum = ~sum(outData[2:]) & 0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id) / 10

    def shutdown(self):
        '''
            Closes the serial port
            And cleans up the mess
            This should be called when the program exits
        '''
        Ax12.port.close()
        Ax12.gpioSet = False

    def move(self, id, position):
        '''
            Moves the specified servo to the specified position as fast as possible
            @param id               The servo id
            @param position         The position to move to
            @return Succes?         returns if the movement was succesfull
            @return IO exception    An exception when the read takes to long(time-out)
        '''
        outData = bytearray([
                Ax12.AX_START, Ax12.AX_START, id,
                Ax12.AX_GOAL_LENGTH, Ax12.AX_WRITE_DATA,
                Ax12.AX_GOAL_POSITION_L, position & 0xff, position >> 8])
        checksum = ~sum(outData[2:]) & 0xff
        outData.append(checksum)
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)
