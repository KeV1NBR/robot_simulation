import rospy
import fcl
import tf


# human setting
body = fcl.Cylinder(0.2, 0.6) #0
head = fcl.Sphere(0.15)       #1
arm = fcl.Cylinder(0.1, 0.25) #2,3,5,6
hand = fcl.Sphere(0.1)        #4,7

# robot setting
Link2 = fcl.Cylinder(0.075, 0.45) #0
Link3 = fcl.Cylinder(0.05, 0.4)   #1
Link4 = fcl.Cylinder(0.05, 0.11)  #2
Link5 = fcl.Cylinder(0.085, 0.43) #3

class Collision:
    def __init__(self, partName, part):
        self.partName = partName
        self.part = part


humanPartIndex = {0 : Collision("body", body), 1 : Collision("head", head), 2 : Collision("shoulder_left", arm), 3 : Collision("arm_left", arm), 4 : Collision("hand_left", hand), 5 : Collision("shoulder_right", arm), 6 : Collision("arm_right", arm), 7 : Collision("hand_right", hand)}

robotPartIndex = {0 : Collision("Link2", Link2), 1: Collision("Link3", Link3), 2 : Collision("Link4", Link4), 3 : Collision("Link5", Link5)}

class CollisionPair:
    def __init__(self, humanNum, humanPart, robotPart):
        self.humanNum = humanNum
        self.humanPartInfo = humanPart
        self.robotPartInfo = robotPart
        self.info = fcl.DistanceResult()

    def update(self):
        if(listener.canTransform('/world', 'human'+str(self.humanNum+1)+'_'+self.humanPartInfo.partName, rospy.Time(0))&listener.canTransform('/world', self.robotPartInfo.partName, rospy.Time(0))):
            (humanTrans,humanRot) = listener.lookupTransform('/world', 'human'+str(self.humanNum+1)+'_'+self.humanPartInfo.partName, rospy.Time(0))
            (robotTrans,robotRot) = listener.lookupTransform('/world', self.robotPartInfo.partName, rospy.Time(0))
            humanTf = fcl.Transform(humanRot, humanTrans)
            robotTf = fcl.Transform(robotRot, robotTrans)
            request = fcl.DistanceRequest()
            humanObject = fcl.CollisionObject(self.humanPartInfo.part, humanTf)
            robotObject = fcl.CollisionObject(self.robotPartInfo.part, robotTf)
            fcl.distance(humanObject, robotObject, request, self.info)

    def getInfo(self):
        return self.info

if __name__ == '__main__':
    rospy.init_node('collision_distance')
    listener = tf.TransformListener()

    pairList = []
    for  robotIt in range(4):
        for num in range(1):
            for humanIt in range(8):
                pairList.append(CollisionPair(num, robotPartIndex[robotIt], humanPartIndex[humanIt]))


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for it in pairList:
            it.update()
            print((it.humanPartInfo.partName) + ' ' + (it.robotPartInfo.partName) + ' : ' + str(it.getInfo().min_distance))
        print('\n\n')
        #(trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
