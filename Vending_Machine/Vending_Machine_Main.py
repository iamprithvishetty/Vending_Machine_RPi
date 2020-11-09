import RPi.GPIO as GPIO
import time
from datetime import datetime
import Adafruit_CharLCD as LCD
import requests
from PIL import Image
import base64
import cv2
import shutil
import sys
import os
import numpy as np
from utils import *
from matplotlib import pyplot as plt
import subprocess
from gtts import gTTS
import serial

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

##Cost of medicine
Money = {1:200,2:250,3:100,4:150} #MRP Rupees

##GSM variables
GSM_conf = False
admin_num = ""
admin_config = False


##Camera Variables
max_val = 8
max_pt = -1
max_kp = 0

##LCD Variables
lcd_rs = 6
lcd_en = 13
lcd_d4 = 19
lcd_d5 = 26
lcd_d6 = 21
lcd_d7 = 20
lcd_backlight = 4 
flag=1

# Specify a 20x4 LCD.
lcd_columns = 20
lcd_rows    = 4

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,lcd_columns, lcd_rows, lcd_backlight)
lcd.clear()
lcd.message('AUTOMATED VENDING')
lcd.set_cursor(0,2)
lcd.message('MACHINE')
print("Automated Vending Machine")

ir_detect = 9

push_button_1 = 17
push_button_2 = 27
push_button_3 = 23
push_button_4 = 24

reset_button = 10

motor_1 = 4
motor_2 = 16
motor_3 = 22
motor_4 = 12

rollerA = 3
rollerB = 11

GPIO.setup(rollerA,GPIO.OUT)
GPIO.setup(rollerB,GPIO.OUT)

GPIO.setup(motor_1,GPIO.OUT)
GPIO.setup(motor_2,GPIO.OUT)
GPIO.setup(motor_3,GPIO.OUT)
GPIO.setup(motor_4,GPIO.OUT)

GPIO.setup(reset_button,GPIO.IN)

GPIO.setup(push_button_1,GPIO.IN)
GPIO.setup(push_button_2,GPIO.IN)
GPIO.setup(push_button_3,GPIO.IN)
GPIO.setup(push_button_4,GPIO.IN)

GPIO.setup(ir_detect,GPIO.IN)

GPIO.output(rollerA,False)
GPIO.output(rollerB,False)
GPIO.output(motor_1,False)
GPIO.output(motor_2,False)
GPIO.output(motor_3,False)
GPIO.output(motor_4,False)

print(GPIO.input(reset_button))

##GSM functions

def send_cmd(cmd,response=None,t=0.5):
            port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=t)
            cmd1 = cmd
            cmd = str.encode(cmd + "\r")
            port.write(cmd)
            print("cmd" + str(cmd))
            rcv = port.readall()
        ##    print('rc: ' + str(rcv))
            rcv = rcv.decode()
            rcv = rcv.strip()
            print ("rcv = ", rcv)
        ##    print(type(rcv))
            if (cmd1 == "AT+CGPSINF=2"):
                return str(rcv)
            
            elif response:
                print (rcv.endswith(response))
                return rcv.endswith(response)


def send_sms(text,num):
    print ("sending sms to ",num)
    send_cmd("AT+CMGF=1",ok)
    if send_cmd("AT+CMGS=\""+num+'\"','>'):
        if send_cmd(text+"\x1a",ok,5):
            print ("sms sent")
        else:
            print ("cant send sms....check your balance")
                
def get_data():
    print ("data available")
    rcv = port.readall().strip()
    rcv = rcv.decode()
    print ("rcv=" , rcv)   
    check_data(rcv)
    port.flushInput()

def check_data(data):
    global admin_num
    global admin_config
            
    if data.find("+CLIP") > 0:
        index1 = data.find('\"') + 1
        index2 = data.find(',') - 1
        number = data[index1:index2]
        print ("receiving call from ",number)
                
        if not admin_config:
            admin_num = number
            admin_config = True
            print ("admin number saved..",admin_num)
            time.sleep(1)
            send_cmd("ATH",ok)
            print ("call cut")
            send_sms("This number is configured as ADMIN..",admin_num)
                    
        elif admin_config :
            print ("configuration already done")
            time.sleep(1)
            send_cmd("ATH",ok)
            print ("call cut")

        else:
            print ("%s already configure.."%admin_num)
            time.sleep(1)
            send_cmd("ATH",ok)
            print ("call cut")

    if data.find("+CMT") > 0:
        index1 = data.find('\"') + 1
        index2 = data.find(',') - 1
        sms_number = data[index1:index2]
        index3 = data.rfind('"') + 1
        sms = data[index3:]
        print ("number: ",sms_number)
        print ("sms: ", sms)

##Vending Machine Functions

def Note_Enter(Medicine):
    global flag
    flag=1
    Medicine= Medicine
    Iteration = 0
    quit_flag=0
    count_out=0
    reset_flag=0
    while GPIO.input(ir_detect) == False and GPIO.input(reset_button)!= False:
        print("Note not detected")
        time.sleep(0.2) #Wait for 200ms
        count_out= count_out+1
        if count_out >= 5*10:
            print("Exit")
            quit_flag = 1 #To check if no note is detected for 10secs
            break
    Money_Needed = Money[Medicine]
    Motor_Flag = 0
    if quit_flag == 0 and GPIO.input(reset_button)!= False:
        Motor_flag =0
        while Motor_flag != 1 and reset_flag!=1 :
            if Iteration > 0:
                idle_count=0
                while GPIO.input(ir_detect)==False:
                    print("Note not detected")
                    print(idle_count)
                    time.sleep(0.2) #Wait for 200ms
                    idle_count=idle_count+1
                    if idle_count >= 5*30:
                        reset_flag = 1
                        break
                    if GPIO.input(reset_button)==False:
                        reset_flag = 1
                        break
                    
            if reset_flag != 1:
                print("Note detected")
                while GPIO.input(ir_detect)!=False and GPIO.input(reset_button)!= False :
                    GPIO.output(rollerA,True)
                    GPIO.output(rollerB,False)
                GPIO.output(rollerA,False)
                GPIO.output(rollerB,False) 
                Money_Now = Camera_Click()
                print(Money_Now,"Money_Now")
                Money_Needed = Money_Needed-Money_Now
                print(Money_Needed,"Money_Needed")
                
                if Money_Now == 0:
                    start = time.time()
                    stop = 0
                    while stop != 1 and GPIO.input(reset_button)!= False:
                        end = time.time()
                        GPIO.output(rollerA,True)
                        GPIO.output(rollerB,False)
                        rotate_time = 6 #Time in secs for rotation
                        if end-start >= rotate_time: 
                            GPIO.output(rollerA,False)
                            GPIO.output(rollerB,False) 
                            stop = 1
                    
                if Money_Needed == 0:
                    Motor_flag = 1
                    start = time.time()
                    stop = 0
                    while stop != 1 and GPIO.input(reset_button)!= False:
                        end = time.time()
                        GPIO.output(rollerA,False)
                        GPIO.output(rollerB,True)
                        rotate_time = 6 #Time in secs for rotation
                        if end-start >= rotate_time: 
                            GPIO.output(rollerA,False)
                            GPIO.output(rollerB,False) 
                            stop = 1
                            
                if Money_Needed > 0:
                    start = time.time()
                    stop = 0
                    while stop != 1 and GPIO.input(reset_button)!= False:
                        end = time.time()
                        GPIO.output(rollerA,False)
                        GPIO.output(rollerB,True)
                        rotate_time = 6 #Time in secs for rotation
                        if end-start >= rotate_time: 
                            GPIO.output(rollerA,False)
                            GPIO.output(rollerB,False) 
                            stop = 1

                    
                if Money_Needed < 0:
                    Money_Needed = Money_Needed+Money_Now
                    start = time.time()
                    stop = 0
                    while stop != 1 and GPIO.input(reset_button)!= False:
                        end = time.time()
                        GPIO.output(rollerA,True)
                        GPIO.output(rollerB,False)
                        rotate_time = 5 #Time in secs for rotation
                        if end-start >= rotate_time: 
                            GPIO.output(rollerA,False)
                            GPIO.output(rollerB,False) 
                            stop = 1
                lcd.clear()
                lcd.set_cursor(0,0)
                lcd.message("Money Entered :")
                lcd.set_cursor(2,1)
                lcd.message("Rs "+str(Money_Now))
                lcd.set_cursor(0,2)
                lcd.message("Money Remaining :")
                lcd.set_cursor(2,3)
                lcd.message("Rs "+str(Money_Needed))
                Iteration= Iteration+1
                #print(Money_Needed)
                
            if Motor_flag == 1:
                Medicine_Motor(Medicine)
                lcd.clear()
                lcd.set_cursor(5,1)
                lcd.message("Thank You")
                send_sms("Medicine "+str(Medicine)+" bought",admin_num)    
            
        
def Camera_Click():
    global max_pt,max_val,max_kp
    capture = cv2.VideoCapture(0)
    ret, test_img = capture.read()
    test_img = cv2.cvtColor(test_img,cv2.COLOR_BGR2GRAY)
    #test_img = read_img('/home/pi/main_test.jpg')
    orb = cv2.ORB_create()
    (kp1, des1) = orb.detectAndCompute(test_img, None)

    # Dataset Folder PATH goes here
    FolderPath="/home/pi/main_dataset"

    #This automatically reads all the images in the dataset
    training_set = os.listdir(FolderPath)
    for i in range(0,len(training_set)):
            training_set[i]=FolderPath+'/'+training_set[i]

    for i in range(0, len(training_set)):
            # train image
            train_img = cv2.imread(training_set[i])

            (kp2, des2) = orb.detectAndCompute(train_img, None)

            # brute force matcher
            bf = cv2.BFMatcher()
            all_matches = bf.knnMatch(des1, des2, k=2)

            good = []
            # give an arbitrary number -> 0.789
            # if good -> append to list of good matches
            for (m, n) in all_matches:
                    if m.distance < 0.789 * n.distance:
                            good.append([m])

            if len(good) > max_val:
                    max_val = len(good)
                    max_pt = i
                    max_kp = kp2

            #print(i, ' ', training_set[i], ' ', len(good))

    if max_val != 8:
            #print(training_set[max_pt])
            #print('good matches ', max_val)

            train_img = cv2.imread(training_set[max_pt])
            img3 = cv2.drawMatchesKnn(test_img, kp1, train_img, max_kp, good, 4)
            
            note = str(training_set[max_pt])[22:len(training_set[max_pt])]
            print('Detected denomination: Rs. ', note)
            #(plt.imshow(img3), plt.show())
            count=0
            for string in note:
                if ord(string)>= 57:
                    break
                count=count+1
            return int(note[0:count])
                    
    else:
            print('No Matches')
            return 0

def Medicine_Motor(Medicine):
    start = time.time()
    stop = 0
    
    if Medicine == 1:
        rotate_motor = motor_1
    elif Medicine == 2:
        rotate_motor = motor_2
    elif Medicine== 3:
        rotate_motor = motor_3
    elif Medicine== 4:
        rotate_motor = motor_4
        
    while stop != 1 :
        end = time.time()
        GPIO.output(rotate_motor,True)
        rotate_time = 5 #Time in secs for rotation
        if end-start >= rotate_time: 
            GPIO.output(rotate_motor,False)
            stop = 1
flag=0         
while True:
    #print(GPIO.input(push_button_1)) #for debugging
    if GSM_conf == False:
        port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
        ok = "OK"
        number = ""
        rcv = None
        sms_sent = True
        main = True
        if main:
            print ("connecting GSM")     
            while True:
                if send_cmd("AT",ok):
                    send_cmd("ATE0",ok)
                    send_cmd("AT+CNMI=2,2,0,0",ok)
                    send_cmd("AT+CGPSPWR=1",ok)
                    send_cmd("AT+CLIP=1",ok)
                    print ("GSM connected")
                    lcd.clear()
                    lcd.set_cursor(0,1)
                    lcd.message("  GSM CONNECTED  ")
                    time.sleep(1)
                    GSM_conf=True
                    break
                else:
                    print ("GSM not connected")
                    lcd.clear()
                    lcd.set_cursor(0,1)
                    lcd.message("  GSM NOT CONNECTED  ")
                    main = False
                    time.sleep(3)
        ##            lcd.clear()
        ##            lcd.message('connect GSM and\nrestart system')
        ##            while GPIO.input(restart) == True:
        ##                None
                    break
            port.flushInput()
            port.flushOutput()
            if main:
                print ("Waiting for admin")
                lcd.clear()
                lcd.message("   WAITING   FOR   ")
                lcd.set_cursor(0,2)
                lcd.message("      ADMIN")
                while not admin_config:
                    time.sleep(0.5)
                    if port.inWaiting() > 0:
                        get_data()
                print('Admin number is configured')
                sms_sent = False
        
    else:
        if flag==1:
            lcd.clear()
            lcd.message('Medicine 1 : 200 Rs')
            lcd.set_cursor(0,1)
            lcd.message('Medicine 2 : 250 Rs')
            lcd.set_cursor(0,2)
            lcd.message('Medicine 2 : 150 Rs')
            lcd.set_cursor(0,3)
            lcd.message('Medicine 2 : 100 Rs')
            flag=0
            
        if GPIO.input(push_button_1)==False: #Check if Push Button 1 is pressed
            lcd.clear()
            lcd.set_cursor(5,1)
            lcd.message('Enter Note')
            print("Button 1 Pressed")
            Note_Enter(1)

        if GPIO.input(push_button_2)==False: #Check if Push Button 2 is pressed
            lcd.clear()
            lcd.set_cursor(5,1)
            lcd.message('Enter Note')
            print("Button 2 Pressed")
            Note_Enter(2)

        if GPIO.input(push_button_3)==False: #Check if Push Button 3 is pressed
            lcd.clear()
            lcd.set_cursor(5,1)
            lcd.message('Enter Note')
            print("Button 3 Pressed")
            Note_Enter(3)
            
        if GPIO.input(push_button_4)==False: #Check if Push Button 4 is pressed
            lcd.clear()
            lcd.set_cursor(5,1)
            lcd.message('Enter Note')
            print("Button 4 Pressed")
            Note_Enter(4)


            
