                      #        x,y
#System size          #        /\
l1 = 0.06             #       /  \
l2 = 0.165            # l2-> /    \<-r2
r1 = 0.060            # l1 ->\_dd_/<-r1
r2 = 0.163            #  |   q1  q2
d  = 0.150 / 2        # Y|
                      #  |____>
                      #    X

from motor_control.AROMotorControl import AROMotorControl
mc = AROMotorControl()

from utils import *
import numpy as np
import time
dt = 0.001
N=300000 #30 seconds
t = time.perf_counter()
i=0
p0 = np.array([0,0.15]) # Adjust this accordingly
R = 0.03 # Adjust this accordingly
for i in range(N):
    # read positions and convert them into radians
    q1 = mc.readPosition(1) * np.pi / 180
    q2 = mc.readPosition(2) * np.pi / 180
    p = fg(q1, q2)
    d = np.linalg.norm(p-p0)
    f = np.array([0.,0.])
    if (d<R):
        f = (p-p0)/d
    tau = J(q1, q2).T @ f
    temp1, current1, speed1, pos1 = mc.applyTorqueToMotor(1, tau[0]*2)
    temp2, current2, speed2, pos2 = mc.applyTorqueToMotor(2, tau[1]*2)
    pos1 = np.radians(pos1)
    pos2 = np.radians(pos2)
    #wait for next control cycle
    t +=dt
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(0.0001)
    if (i%100==0):
        #print(f"q={q}")
        #print(f"p=fk_delta(q)={p}")
        print(f"f={f}, tau={tau}")

