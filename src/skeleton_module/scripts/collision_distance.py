import rospy
import fcl
import tf
import numpy as np
import time

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
    def __init__(self, humanPartNum, robotPartNum):
        self.humanPartNum = humanPartNum
        self.robotPartNum = robotPartNum
    def update(self):
        info = fcl.DistanceResult()

        humanTf = humanTransforms[self.humanPartNum]
        robotTf = robotTransforms[self.robotPartNum]
        request = fcl.DistanceRequest()
        humanObject = fcl.CollisionObject(humanPartIndex[self.humanPartNum].part, humanTf)
        robotObject = fcl.CollisionObject(robotPartIndex[self.robotPartNum].part, robotTf)
        fcl.distance(humanObject, robotObject, request, info)

        return info


robotTransforms = []
humanTransforms = []

def updateTransforms():
    deg90 = np.array([0, 0.7071068, 0, 0.7071068])
    global robotTransforms
    global humanTransforms
    listener = tf.TransformListener()
    ret = True
    listener.waitForTransform("/world", "/LINK2", rospy.Time(), rospy.Duration.from_sec(1))
    if(listener.canTransform('/world', '/LINK2', rospy.Time(0))):
        robotTransforms.clear()
        (trans,rot) = listener.lookupTransform('/world', '/LINK2', rospy.Time(0))
        robotTransforms.append(fcl.Transform(rot, trans))
        (trans,rot) = listener.lookupTransform('/world', '/LINK3', rospy.Time(0))
        robotTransforms.append(fcl.Transform(rot, trans))
        (trans,rot) = listener.lookupTransform('/world', '/LINK4', rospy.Time(0))
        robotTransforms.append(fcl.Transform(rot, trans))
        (trans,rot) = listener.lookupTransform('/world', '/LINK5', rospy.Time(0))
        robotTransforms.append(fcl.Transform(rot, trans))
    else:
        ret = False

    if(listener.canTransform('/world', '/Spine_Naval0', rospy.Time(0))):
        humanTransforms.clear()

        (trans1,rot1) = listener.lookupTransform('/world', 'Spine_Naval0', rospy.Time(0))
        (trans2,rot2) = listener.lookupTransform('/world', 'Spine_Chest0', rospy.Time(0))
        trans1 = np.array(trans1)
        trans2 = np.array(trans2)

        humanTransforms.append(fcl.Transform(np.array(rot1)*deg90, (trans1+trans2)/2))

        (trans1,rot1) = listener.lookupTransform('/world', 'Ear_Right0', rospy.Time(0))
        (trans2,rot2) = listener.lookupTransform('/world', 'Ear_Left0', rospy.Time(0))
        trans1 = np.array(trans1)
        trans2 = np.array(trans2)
        humanTransforms.append(fcl.Transform(rot1, (trans1+trans2)/2))

        (trans1,rot1) = listener.lookupTransform('/world', 'Shoulder_left0', rospy.Time(0))
        (trans2,rot2) = listener.lookupTransform('/world', 'Elbow_left0', rospy.Time(0))
        trans1 = np.array(trans1)
        trans2 = np.array(trans2)
        humanTransforms.append(fcl.Transform(np.array(rot1)*deg90, (trans1+trans2)/2))

        (trans1,rot1) = listener.lookupTransform('/world', 'Wrist_left0', rospy.Time(0))
        trans1 = np.array(trans1)
        humanTransforms.append(fcl.Transform(np.array(rot2)*deg90, (trans1+trans2)/2))

        (trans2,rot2) = listener.lookupTransform('/world', 'Handtip_left0', rospy.Time(0))
        trans2 = np.array(trans2)
        humanTransforms.append(fcl.Transform(rot1, (trans1+trans2)/2))


        (trans1,rot1) = listener.lookupTransform('/world', 'Shoulder_right0', rospy.Time(0))
        (trans2,rot2) = listener.lookupTransform('/world', 'Elbow_right0', rospy.Time(0))
        trans1 = np.array(trans1)
        trans2 = np.array(trans2)
        humanTransforms.append(fcl.Transform(np.array(rot1)*deg90, (trans1+trans2)/2))

        (trans1,rot1) = listener.lookupTransform('/world', 'Wrist_right0', rospy.Time(0))
        trans1 = np.array(trans1)
        humanTransforms.append(fcl.Transform(np.array(rot2)*deg90, (trans1+trans2)/2))


        (trans2,rot2) = listener.lookupTransform('/world', 'Handtip_right0', rospy.Time(0))
        trans2 = np.array(trans2)
        humanTransforms.append(fcl.Transform(rot1, (trans1+trans2)/2))

    else:
        ret = False

    return ret


if __name__ == '__main__':
    rospy.init_node('collision_distance')

    pairList = []
    for  robotIt in range(4):
        for num in range(1):
            for humanIt in range(8):
                pairList.append(CollisionPair(humanIt, robotIt))


    #rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        start = time.time()
        if updateTransforms():

            rospy.loginfo('it')
            for it in pairList:
                info = it.update()
                print((humanPartIndex[it.humanPartNum].partName) + ' ' + (robotPartIndex[it.robotPartNum].partName) + ' : ' ,info.min_distance)
            print('\n\n')
            end = time.time()
            print(start-end, 's')
        else:
            print('cant get tf')

     #   rate.sleep()
