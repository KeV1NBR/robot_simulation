#import rospy
import fcl
#import numpy

# human setting
head = fcl.Sphere(0.15)
hand = fcl.Sphere(0.1)
body = fcl.Cylinder(0.2, 0.6)
arm = fcl.Cylinder(0.1, 0.25)

# robot setting
Link1 = fcl.Cylinder(0.075, 0.45)
Link2 = fcl.Cylinder(0.05, 0.4)
Link3 = fcl.Cylinder(0.05, 0.11)
Link4 = fcl.Cylinder(0.085, 0.43)
