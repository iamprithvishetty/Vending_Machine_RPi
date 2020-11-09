import cv2
import numpy
import os

Path = "/home/pi/main_dataset"
directory= os.listdir(Path)
for file in range(0,len(directory)-1):
    directory[file]=Path+"/"+directory[file]
print(directory)

for file in range(0,len(directory)-1):
    Image = cv2.imread(directory[file],0)
    cv2.imwrite(directory[file],Image)
