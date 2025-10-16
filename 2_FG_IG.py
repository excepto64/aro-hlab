from motor_control.AROMotorControl import AROMotorControl
import numpy as np

mc = AROMotorControl()
mc.setZero(1)
mc.setZero(2)

#System size          #        /\
l1 = 0.06             #       /  \
l2 = 0.165            # l2-> /    \<-r2
r1 = 0.060            # l1 ->\_dd_/<-r1
r2 = 0.163            #  |   q1  q2
d  = 0.150 / 2        # Y|
                      #  |____>
                      #    X