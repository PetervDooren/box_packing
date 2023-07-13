from PyKDL import Frame, Vector

class BoxRegion:
    """
    Create a region with its center at (x,y,z) and with dimensions (dx, dy, dz)
    """
    def __init__(self, frame, dx, dy, dz):
        self.center = frame
        self.min = Vector(frame.p.x()-dx/2, frame.p.y()-dy/2, frame.p.z()-dz/2)
        self.max = Vector(frame.p.x()+dx/2, frame.p.y()+dy/2, frame.p.z()+dz/2)
        self.dx = dx
        self.dy = dy
        self.dz = dz

    def getCenterFrame(self):
        return self.center
    
    def getCenterPoint(self):
        return self.center.p
    
    def getRotation(self):
        return self.center.M

    def getMin(self):
        return self.min
    
    def getMax(self):
        return self.max
