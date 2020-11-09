import cv2
import numpy
capture =cv2.VideoCapture(0)
img_no_pos = 64
img_bg = 0
Money_Value = ""

while True:
    ret,image = capture.read()
    cv2.imshow("Screenshot",image)

    if cv2.waitKey(1) & 0xFF == ord('p'):
        cv2.imwrite("/home/pi/Negative/"+Money_Value+str(img_bg)+".jpg",image)
        print(img_bg)
        img_bg = img_bg+1
        
        
