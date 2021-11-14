
import serial
from datetime import datetime
from time import sleep
import pynmea2 as pn
import pywinauto

f = open("log.txt","a")

serialPort = serial.Serial(
    port="COM1", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)

sm = pywinauto.application.Application().start("C:\\tools\soundmodem113\soundmodem.exe")

t = sm.TForm1.Edit

prelen = len(t.texts())
preline = ""
while(True):
    
    l=len(t.texts())
    if(l>=3):
        s = t.texts()[l-2]
        if(s == "" and t.texts()[l-3][0]=="$"):
            s = t.texts()[l-3]
        elif(s==""):
            continue
        if(l>prelen or(preline!=s)):
            print(s)
            if(s[0]=="$"):
                msg = pn.parse(s)
                print(f'LAT:{msg.lat}{msg.lat_dir}\tLON:{msg.lon}{msg.lon_dir}\tALT:{msg.altitude}\n')
            f.write(datetime.now().strftime("%H:%M:%S")+"\t"+s+"\n")
            prelen = l
            preline = s