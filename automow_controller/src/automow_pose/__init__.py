import math

class automow_pose:
    def __init__(self,x=0.0,y=0.0,z=0.0,alpha=0.0):
        self.x = float(x) 
        self.y = float(y) 
        self.z = float(z) 
        self.alpha = float(alpha) 

        r = math.sqrt(2-(2*math.cos(self.alpha)))

        if r==0:
            self.qw = 0
        else:
            self.qw = math.sin(self.alpha)/r

        self.qx = 0
        self.qy = 0
        self.qz = r/2

        if(self.qw < 1):
            self.qw *= -1
            self.qz *= -1

        self.cos_alpha = math.cos(self.alpha)
        self.sin_alpha = math.sin(self.alpha)

    def get_alphad(self): 
        return math.degrees(self.alpha)

    def update_alpha(self):
        self.cos_alpha = self.qw * self.qw + self.qx * self.qx \
                - self.qy * self.qy - self.qz * self.qz
        self.sin_alpha = 2*self.qw*self.qz + 2*self.qx*self.qy
        self.alpha = math.atan2(self.sin_alpha,self.cos_alpha)
        return

    def __str__(self):
        string = "x: %f, y: %f, z: %f \n"%(self.x,self.y,self.z)
        string+= "qx: %f, qy: %f, qz: %f, qw: %f"%(self.qx,self.qy,self.qz,self.qw)
        string+="\n"
        return string


