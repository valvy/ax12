#!/usr/bin/env python3
import sys
import ax12 as ax12
from time import sleep

def help():
    print("Commands")
    print("<move|m> to move servo")
    print("<rVolt> to read voltage")
    print("<rPos> to read position")
    print("<setanglelimit> to change angle limit")
    print("<ping> checks for the connected servo's")

ax = ax12.Ax12()

def execute(id,cmd,param):
    if cmd == "move" or cmd == "m":
        if(param == -1):
            param = int(input('rotation:'))
        if param > 1023:
            print("To high")
        elif param < 0:
            print("To low")
        else:
            print(ax.move(int(id),param))
    elif cmd == "rvolt":
        print(ax.readVoltage(id))
    elif cmd == "rpos":
        print(ax.readPosition(id))
    elif cmd == "help":
        help()
    elif cmd == "ping":
         print(ax.ping(id))
    elif cmd == "learn":
        param = int(input("min : "))
        paramTwo = int(input("max : "))
        print(ax.learnServos(param,paramTwo, lambda x: print(x)))
    elif cmd == "setanglelimit":
        param = int(input("cwLimit : "))
        paramTwo = int(input("ccwLimit : "))
        print(ax.setAngleLimit(id,param,paramTwo))
    elif cmd == "rtemp":
        print(ax.readTemperature(id))
    elif cmd == "setspeed":
        param = int(input("Position :"))
        paramTwo = int(input("speed : "))
        print(ax.moveSpeed(id,param,paramTwo))
    elif cmd == "setshutdown":
        param = int(input("on/off : "))
        print(ax.setShutdownAlarm(id,param))
    else:
        help()

def main(args):
    try:
        if len(args) > 0:
            if args[0].lower() == "move" or args[0].lower() == "m":
                if len(args) > 2:
                    execute(int(args[1]), args[0].lower(), int(args[2]))
                    return
            elif args[0].lower() == "ping":
                if len(args) > 1:
                    print(execute(int(args[1]),args[0].lower(),-1))
                    return
            elif args[0].lower() == "learn":
                execute(-1,args[0].lower(),-1)
            elif args[0].lower() == "setanglelimit":
                if len(args) > 1:
                    execute(int(args[1]), args[0].lower(), -1)
                    return
            elif args[0].lower() == "setspeed":
                if len(args) > 1:
                    execute(int(args[1]),args[0].lower(), -1)
        else:
            print("test program for ax12 servo's")
            print("Please press a number to move the servo 1")
            print("to stop please use control c")
            print("for help press after id help")
            while True:
                try:
                    com = input("Servo ID: ")
                    id = int(com)
                    cmd = input("Command: ").lower()
                    execute(id,cmd,-1)
                except ValueError:
                    print("Not an number")
                except ax12.Ax12.timeoutError as e:
                    print(str(e))
    except ax12.Ax12.timeoutError:
        print("timeout")
    except( KeyboardInterrupt):
        ax.shutdown()
    except ValueError:
        print("Invalid input")

if  __name__ =="__main__":
     main(sys.argv[1:])
