import numpy as np

class Projection(object):

    def __init__(self, reference, anchors):
        self.reference = reference
        self.anchors = anchors
        self.rx = (reference[4] - reference[0])/(anchors[4] - anchors[0])
        self.ry = (reference[3] - reference[1])/(anchors[3] - anchors[1])

    def SetConstantCorrection(self, x, y):
        self.cc = [x, y]

    def SetVariableCorrection(self, x, rx, y, ry):
        self.vc = [x, rx, y, ry]

    def Project(self, x, y):
        return int(x*self.rx) + self.cc[0], int(y*self.ry) + self.cc[1]
