import cv2
import numpy as np
cap = cv2.VideoCapture(0)

while True:  
    ret, img = cap.read()
    cv2.imshow("Masked Image", img)
##    cv2.normalize()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('/home/pi/main_dataset/10D.jpg',img)
        break
