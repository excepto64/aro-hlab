from motor_control.AROMotorControl import AROMotorControl
from template import run_until
import matplotlib.pyplot as plt

anglevalues =[]

mc = AROMotorControl()

def clip(output):
    outabs = abs(output)
    if outabs < 1e-4:
        return 0
    clipped = max(min(outabs, 0.1), 0.023)
    return clipped if output > 0 else -clipped

class PController:
    
    def __init__(self, Kp):
        self.Kp = Kp
                      
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
                diff = diff + 360
        if (current + diff) % 360 == target:
                return diff
        else:
                return -diff
        
    def compute(self, target, current):
        error = self.shortest_path_error(target, current)
        output = self.Kp*error
        return clip(output)
        
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
    
    time_values = [i * dt for i in range(len(anglevalues))]
    # Plotting the angle values
    plt.figure(figsize=(10, 5))
    plt.plot(time_values, anglevalues, marker='o', linestyle='-')
    plt.axhline(y=target, color='r', linestyle='--', label='Target Value')  # Add horizontal line for target
    plt.title('Motor Angle Values Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle Values (% 2Pi)')
    plt.grid()
    plt.show()




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
    


dt = 0.005
N = int(2. / dt)

try:    
    pdc = PDController(0.00016,0.001*dt)
    goTo(pdc, 200, time = 1.5)
except KeyboardInterrupt:
    print("KeyboardInterrupt received, stopping motors...")
except Exception as e:
    print(f"an error occurred: {e}")
finally:
    mc.applyTorqueToMotor(1, 0)
    mc.applyTorqueToMotor(2, 0)
    print("motors stopped!")
    
        



# print(mc.readPosition(motorid=1))
# print(mc.readPosition(motorid=2))


# dt = 0.005
# N = int(2. / dt)

# def store_values_and_apply_torques(motorid, torque):
#     global anglevalues
#     mc.applyTorqueToMotor(motorid=motorid, torque=torque)
#     anglevalues+= [mc.readPosition(motorid)]
    
# try:
#     run_until(store_values_and_apply_torques, N=N, dt=0.005, motorid=1, torque=0.02)
# except KeyboardInterrupt:
#     print("KeyboardInterrupt received, stopping motors...")
# except Exception as e:
#     print(f"an error occurred: {e}")
# finally:
#     mc.applyTorqueToMotor(1, 0)
#     mc.applyTorqueToMotor(2, 0)
#     print("motors stopped!")
    

    
# time_values = [i * dt for i in range(len(anglevalues))]
    
# # Plotting the angle values
# plt.figure(figsize=(10, 5))
# plt.plot(time_values, anglevalues, marker='o', linestyle='-')
# plt.title('Motor Angle Values Over Time')
# plt.xlabel('Time (s)')
# plt.ylabel('Angle Values (% 2Pi)')
# plt.grid()
# plt.show()