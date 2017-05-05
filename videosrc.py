import numpy as np
import cv2
lower_red = np.array([80, 100, 50])
upper_red = np.array([255, 255, 180])
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_green = np.array([60, 90, 50])
upper_green = np.array([100, 255, 255])

class VideoSrc(object):
    

    def __init__(self, width, height, src=0):
        self.width = width
        self.height = height
        self.cols = [0 for i in range(width)]
        self.rows = [0 for i in range(height)]
        self.camera = cv2.VideoCapture(src)
        self.camera.set(3, width)
        self.camera.set(4, height)

    def Read(self):
        ret, frame = self.camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow("VideoSrc", frame)

    def FindTarget(self):
        col, row = 0, 0
        ret, frame = self.camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.array(cv2.inRange(hsv, lower_red, upper_red))
        
        self.cols = mask.sum(axis=0)
        self.rows = mask.sum(axis=1)
        center = sum(self.cols)/2
        
        temp = 0
        for c in range(self.width):
            temp += self.cols[c]
            if(temp > center):
                col = c
                break
            
        temp = 0
        for r in range(self.height):
            temp += self.rows[r]
            if(temp > center):
                row = r
                break
            
        cv2.circle(frame, (col,row), 20, (255,0,0), 1)
        return frame, row, col
        

    def FindReferences(self):
        anchors = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        ret, frame = self.camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_green, upper_green)
        anchors[0], anchors[1] = self.FindAnchor(frame, mask, 0, 100, 0, 100)
        anchors[2], anchors[3] = self.FindAnchor(frame, mask, 0, 100, 300, 500)
        anchors[4], anchors[5] = self.FindAnchor(frame, mask, 260, 360, 0, 100)
        anchors[6], anchors[7] = self.FindAnchor(frame, mask, 260, 360, 300, 500)
        return frame, anchors
        
    def FindAnchor(self, frame, mask, startrow, endrow, startcol, endcol):
        mass = 0
        for c in range(startcol, endcol):
            for r in range(startrow, endrow):
                mass = mass + 1 if mask[r][c] > 0 else mass
        center = mass*0.5
    
        for c in range(startcol, endcol):
            for r in range(startrow, endrow):
                center = center - 1 if mask[r][c] > 0 else center
                if(center <= 0):
                    cv2.circle(frame, (c, r), 20, (0,0,255), 1)
                    return r, c
        return 0, 0

    def ShowAnchors(self, frame):
        for i in range(4):
            cv2.circle(frame, (anchors[(i*2)+1], anchors[i*2]), 20, (0,0,255), 1)

    def Destroy(self):
        self.camera.release()
        cv2.destroyAllWindows()

    
