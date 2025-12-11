from motor_control.AROMotorControl import AROMotorControl
import numpy as np
import matplotlib.pyplot as plt
import time
from template import run_until

# Set numpy printing options for better debugging
np.set_printoptions(precision=6, suppress=True, linewidth=120)

#System size          #        /\
l1 = 0.06             #       /  \
l2 = 0.165            # l2-> /    \<-r2
r1 = 0.060            # l1 ->\_dd_/<-r1
r2 = 0.163            #  |   q1  q2
d  = 0.150 / 2        # Y|
                      #  |____>
                      #    X

# Points of motors - origin is between motors
ML = np.array([-d, 0])
MR = np.array([d, 0])

def read_fg_loop(mc, dt, T):
    xs, ys, xys = [], [], []
    t = time.perf_counter()
    N = int(T/dt)
    wait = 1. / 1e4
    for i in range(N):
        t +=dt
        q1,q2 = mc.readPosition(1), mc.readPosition(2)
        xy = fg(q1, q2) 
        xs.append(xy[0])
        ys.append(xy[1])
        xys.append(xy)
        time.sleep(dt)
        # while(time.perf_counter()-t<dt):
        #     pass
        #     time.sleep(wait)
    plt.scatter(xs, ys, s=1)
    plt.savefig("fig1.png")
    plt.show()
    return xys

def fg(q1, q2, positive=True):

    # Angles (from facing east, counter-clockwise)
    thetaL = np.mod(q1 + 90, 360)
    thetaR = np.mod(q2 + 90, 360)
    # Intermediate joints
    L = ML + l1 * np.array([np.cos(np.radians(thetaL)), np.sin(np.radians(thetaL))])
    R = MR + r1 * np.array([np.cos(np.radians(thetaR)), np.sin(np.radians(thetaR))])

    # Distance between intermediate joints
    m = np.linalg.norm(R-L)
    u = (R-L)/m

    x = (l2**2 - r2**2 + m**2) / (2*m)
    y  = np.sqrt(l2**2 - x**2)
    v = np.array([-u[1], u[0]])

    if positive:
        pen = L + x*u + y*v
    else:
        pen = L + x*u - y*v
    return pen

def ig(xy):
    lenR = np.linalg.norm(xy - MR)
    lenL = np.linalg.norm(xy - ML)#
    #print(lenL, lenR)
    alphaR = np.arccos((r1**2 - r2**2 + lenR**2)/(2*r1*lenR))
    betaR = np.arctan2(xy[1], -xy[0] + d)
    betaL = np.arccos((l1**2 - l2**2 + lenL**2)/(2*l1*lenL))
    alphaL = np.arctan2(xy[1], xy[0] + d)
    q1 = alphaL + betaL
    q2 = np.pi - alphaR - betaR
    print(np.degrees((alphaL, betaL, alphaR, betaR)))
    return np.degrees(np.array([q1-np.pi/2, q2-np.pi/2]))

def clip(output):
    outabs = abs(output)
    if outabs < 1e-4:
        return 0
    clipped = max(min(outabs, 0.1), 0.023)
    return clipped if output > 0 else -clipped

def goTo(controller, target, time = 1., dt = 0.005, motorid =1):
    anglevalues =[]
    N = (int)(time / dt)
    
    def oneStep():
        nonlocal anglevalues
        currentAngle = mc.readPosition(motorid)
        anglevalues+=[currentAngle]
        tau = controller.compute(target,currentAngle)
        mc.applyTorqueToMotor(motorid,tau)   
        
    run_until(oneStep, N=N, dt=dt)
    
    mc.applyTorqueToMotor(1, 0)
    mc.applyTorqueToMotor(2, 0)

class PDController:
    
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0
        self.positions = []
        
    
    def reset(self):
        self.positions = []
                      
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
                diff = diff + 360
        if (current + diff) % 360 == target:
                return diff
        else:
                return -diff
        
    def compute(self, target, current, dt=0.005):
        self.positions.append(current)
        error = self.shortest_path_error(target, current) 
        d_error = 0
        if (len(self.positions) > 2):       
                d_error = (self.positions[-1] - self.positions[-3]) / (2*dt)
        output = self.Kp*error - self.Kd * d_error
        return clip(output)


try:
    mc = AROMotorControl()
    # mc.setZero(1)
    # mc.setZero(2)
    dt = 0.01
    T = 2
    pdc = PDController(0.00036,0.001*dt)

    print('Reading input')
    # xys = read_fg_loop(mc, dt, T)

    # print('Starting motors in 5!')
    # time.sleep(5)
    # print('motors starting')
    # for xy in xys:
    #     q1q2 = ig(xy)
    #     goTo(pdc, q1q2[0], dt*10, motorid=1)
    #     goTo(pdc, q1q2[1], dt*10, motorid=2)
    #     time.sleep(dt)
    # q1,q2 = mc.readPosition(1), mc.readPosition(2)
    # q1, q2 = 31.31, 68.85
    # xy = fg(q1, q2)
    # print(xy)
    # q1q2 = ig(xy)
    # print(q1, q2, '->', q1q2)

except KeyboardInterrupt:
    print("KeyboardInterrupt received, stopping motors...")
except Exception as e:
    print(f"an error occurred: {e}")
finally:
    mc.applyTorqueToMotor(1, 0)
    mc.applyTorqueToMotor(2, 0)
    print("motors stopped!")