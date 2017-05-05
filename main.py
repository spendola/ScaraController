import numpy as np
import cv2
import datetime
import serialcomm
import videosrc
import projection
import sys

def main():

    # Initialize Serial to Arduino
    arduino = serialcomm.SerialComm("/dev/cu.usbmodem1421", 2)

    # Initialize Camera
    anchors = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    references = np.array([-9.5, 7.5, -9.5, 21.5, 9.5, 7.5, 9.5, 21.5])
    
    print("Finding Camera Reference ...")
    camera = videosrc.VideoSrc(640, 360, 1)
    for i in range(30):
        frame, temp = camera.FindReferences()
        for i in range(8):
            anchors[i] += temp[i]
        cv2.imshow("VideoSrc", frame)
        cv2.waitKey(25)
    mapping = projection.Projection(references, anchors/30)
    mapping.SetConstantCorrection(0, -9)

    # Beging Tracking Objects
    counter = 0
    armIsReady = True
    while(True):
        frame, row, col = camera.FindTarget()
        cv2.imshow("VideoSrc", frame)
        if(cv2.waitKey(5) & 0XFF == ord('q')):
            break
        
        if(counter > 75 and armIsReady):
            x, y = mapping.Project(col, row)
            armIsReady = False
            arduino.WritePosition(y, x)
            
            
        if not armIsReady:
            if(arduino.Read() == "c"):
                counter = 0
                armIsReady = True
                print("Move Completed")
                
        counter = counter + 1

    # Cleanup
    camera.Destroy()

if __name__ == '__main__':
    main()
