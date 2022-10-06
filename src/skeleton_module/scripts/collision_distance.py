import rospy
import fcl
import tf

import numpy as np


class CollisionPair:
    def __init__(self):
        self.humanName = ""
        self.humanPartName = ""
        self.robotPartName = ""
        self.info = fcl.DistanceResult()

    def getInfo(self):
        return self.info


# human setting
body = fcl.Cylinder(0.2, 0.6)
head = fcl.Sphere(0.15)
arm = fcl.Cylinder(0.1, 0.25)
hand = fcl.Sphere(0.1)

# robot setting
Link1 = fcl.Cylinder(0.075, 0.45)
Link2 = fcl.Cylinder(0.05, 0.4)
Link3 = fcl.Cylinder(0.05, 0.11)
Link4 = fcl.Cylinder(0.085, 0.43)


T = np.array([1.0, 0, 0])
transform = fcl.Transform(T)
human1_head = fcl.CollisionObject(head)
human1_body = fcl.CollisionObject(body, transform)

request = fcl.DistanceRequest()
result = fcl.DistanceResult()

ret = fcl.distance(human1_head, human1_body, request, result)
print(result.min_distance)
print(result.nearest_points)

if __name__ == '__main__':
    rospy.init_node('collision_distance')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
