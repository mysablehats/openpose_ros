#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image #, RegioOfInterest
from ar_msgs.msg import Joint, Person, PersonArray
from geometry_msgs.msg import Polygon
from std_msgs.msg import Float32MultiArray, Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from copy import deepcopy

NODEPTH = True

class MyPoint:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

        self.confidence = None
        if NODEPTH:
            self.zeroz()
    def zeroz(self):
        self.z = 0
    def zeroit(self):
        self.x = 0
        self.y = 0
        self.z = 0

        self.confidence = 0
    def randint(self):
        self.x = np.random.random()
        self.y = np.random.random()
        self.z = np.random.random()
        self.confidence = np.random.random()

def mycolor(N):
    return ((N*103)%256,(N*11)%256,(N*199)%256)
def genAU(N):
    AU = []
    for i in range(N):
        apoint = MyPoint()
        apoint.randint()
        AU.append(deepcopy(apoint))
    return AU

def rematchcentroids(SK,A,U):
    ### first let's remove the elemetary solutions:
    if (len(U) ==1 and len(A) ==1) or len(U) == 0:
        ##nothing to be matched
        return SK
    rospy.logdebug('something to match!')
    assert len(SK) == len(U) ### they need to be the same length or I won't be able to match them!
    newsk = [[] for i in range(max([len(A),len(U)]))] ## hmmm this will continue to grow?
    mydistmat = 500*np.ones([len(A),len(U)]) ### actually should be 'infinity' or a large number
    for i,Ai in enumerate(A):
        for j,Uj in enumerate(U):
            ### if I ever put depth..
            mydistmat[i,j] = math.sqrt((Ai.x - Uj.x)*(Ai.x - Uj.x) + (Ai.y - Uj.y)*(Ai.y - Uj.y) + (Ai.z - Uj.z)*(Ai.z - Uj.z))

    ### maybe there should be also some distance threshold so that I don't try to match really distant people. afff...
    if len(A)>len(U):
        ##some people will disappear!
        rospy.logdebug('A is bigger than U. People will disappear!')

    elif len(U)>len(A):
        ##some people will disappear!
        rospy.logdebug('U is bigger than A. New people will appear!')

            # U(col) = A(lin)
    else: ## len(U) == len(A):
        pass
        #all is good? or new people should appear and some people should disappear???
    inda = range(len(A))
    indu = range(len(U))
    while(mydistmat.shape[0]>0 and mydistmat.shape[1]>0 ):
        print(mydistmat)
        lin,col = ind = np.unravel_index(np.argmin(mydistmat, axis=None), mydistmat.shape)
        #
        # lin = mydistmat.argmin(0).min()
        # col = mydistmat.min(0).argmin() ## this is the closest match so far
        realindlin = inda[lin]
        realindcol = indu[col]
        print("matching A: %d to U:%d "%(realindlin,realindcol))
        newsk[realindlin] = SK[realindcol]
        #print(newsk)
        ## SK.pop(col)
        ### now delete the col and lin
        newmat = np.delete(mydistmat,lin,0)
        mydistmat = np.delete(newmat,col,1)
        print(inda.pop(lin))
        print(indu.pop(col))
        ### if I delete i will need to keep track of the indexes!
        ### now delete the col and lin
        ## newmat = np.delete(mydistmat,lin,0)
        ## mydistmat = np.delete(newmat,col,1)
        print(mydistmat.shape)
    ### i think i need to add all of the indu indexes remaining...
    print('remaining indexes %s'%indu)
    newsk.extend([SK[i] for i in indu ])
    return filter(None, newsk) #, mydistmat
    # print newsk
    # return mydistmat
    # return SK

class ShowSkel:
    def __init__(self,polyOrArr):
        rospy.init_node('showskel_node', anonymous=True, log_level=rospy.DEBUG)
        image_topic = 'im_in'# rospy.get_param('~image_topic','camera/rgb/image_color')
        rospy.loginfo('Input image topic being read as background image: {}'.format(image_topic))
        ## Acentroids are the previous known centroids for skeletons I found already. Ucentroids are the new from current frame that I need to match with the old ones.
        self.Acentroids = []
        self.Ucentroids = []
        self.confidence_treshold = 0.5
        if polyOrArr == 'poly':
            self.hss =  rospy.Subscriber('my_topic', Polygon, self.updateskelPoly)
            self.SK = []
            Polygon()
            self.SK.append(SK.points) ### this is just a list. i could initialize it to an empty list...
        elif polyOrArr == 'array':
            self.hss =  rospy.Subscriber('skel', Float32MultiArray, self.updateskelArray)
            self.SK = []
        self.cvbridge = CvBridge()
        #maybe this is a good time for a wait_for_message; to initialize this thing correctly
        rospy.logdebug('waiting for image topic to publish first image so I can initialize this guy ')
        self.bg = self.cvbridge.imgmsg_to_cv2( rospy.wait_for_message(image_topic, Image), 'bgr8')
        self.bgstack = []
        self.bgstacksize = rospy.get_param('~bgstacksize',20) ### adjust this to the amount of delay we have.
        assert self.bgstacksize > 1
        self.iss = rospy.Subscriber(image_topic, Image, self.updatebg)
        self.ipp = rospy.Publisher('skels_im', Image, queue_size=1)
        self.rois = rospy.Publisher('rois', PersonArray, queue_size=1)
        self.mainskelcolor = (0,255,0)
        self.otherskelscolor = (255,0,0)
        # self.bg = np.array([],'uint8')
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        rospy.loginfo('ShowSkel initialized. ')
        self.markersize = 10
        self.myrate = rospy.Rate(10)

        self.numpersons = 1
        self.numbodyparts = 18

    def run(self):
        rospy.loginfo('Running. ')
        while not rospy.is_shutdown():
            self.pubimage()
            self.myrate.sleep()

    def updatebg(self,data):
        # rospy.logdebug('updatebg called')
        ###let's keep a stack of N frames
        self.bgstack.append(self.cvbridge.imgmsg_to_cv2(data, 'bgr8'))
        if len(self.bgstack) > self.bgstacksize:
            self.bgstack.pop(0)
        self.bg = self.bgstack[0]
        # self.bg = self.cvbridge.imgmsg_to_cv2(data, 'bgr8')

    def updateskelPoly(self,data):
        # rospy.logdebug('updateskelPoly called')
        self.SK = data.points

    def updateskelArray(self,data):
        # rospy.logdebug('updateskelArray called')
        ###not flexible. needs to have person, bodyparts and xyconfidence:
        for dimi in data.layout.dim:
            if dimi.label == 'person':
                self.numpersons = dimi.size
                rospy.logdebug('persons: %d'%self.numpersons)
                # rospy.logdebug(type(self.numpersons))
            if dimi.label == 'bodyparts':
                self.numbodyparts = dimi.size
                self.personstride = dimi.stride
                # rospy.logdebug(self.numbodyparts)
                # rospy.logdebug(type(self.numbodyparts))
                # rospy.logdebug(self.personstride)
                # rospy.logdebug(type(self.personstride))
            if dimi.label == 'xyconfidence':
                self.xyconfidence = dimi.size
                # rospy.logdebug(self.xyconfidence)
                # rospy.logdebug(type(self.xyconfidence))
                assert self.xyconfidence == 3 ##it has to be 3.

        self.SK = []
        self.Acentroids = self.Ucentroids
        self.Ucentroids = []
        debugnumpeople = 0
        for Nperson in range(self.numpersons):
            personchunkstart =   int(Nperson*self.personstride)
            # rospy.logdebug(personchunkstart)
            thisPersonData = data.data[personchunkstart:personchunkstart+self.personstride]
            thisPerson = []
            thisCentroid = MyPoint()
            thisCentroid.zeroit()
            for bodypart in range(self.numbodyparts):
                thisBodypart = MyPoint()
                thisBodypart.x =            thisPersonData[bodypart*self.xyconfidence]
                thisBodypart.y =            thisPersonData[bodypart*self.xyconfidence+1]
                thisBodypart.confidence =   thisPersonData[bodypart*self.xyconfidence+2]
                thisPerson.append(thisBodypart)
                ###calculating the centroid:
                if thisBodypart.confidence > self.confidence_treshold: ### will try avoiding using bad bodyparts to compute centroid
                    thisCentroid.x += thisBodypart.x
                    thisCentroid.y += thisBodypart.y
                    thisCentroid.z += thisBodypart.z
                    thisCentroid.confidence += thisBodypart.confidence ### does this make sense?
            thisCentroid.x = thisCentroid.x/self.numbodyparts
            thisCentroid.y = thisCentroid.y/self.numbodyparts
            thisCentroid.z = thisCentroid.z/self.numbodyparts
            thisCentroid.confidence = thisCentroid.confidence/self.numbodyparts
            self.SK.append(deepcopy(thisPerson))
            self.Ucentroids.append(deepcopy(thisCentroid))
            debugnumpeople +=1
            rospy.logdebug('added a person %d'%debugnumpeople)
        ### last but not the least, I need to reorder SK to match the centroids
        self.SK = rematchcentroids(self.SK,self.Acentroids,self.Ucentroids)
        # rospy.loginfo(self.SK)
    def pubimage(self):
        # rospy.logdebug('pubinfo called')
        debugnumpeople = 0
        #print(type(self.bg    ))
        firstskel = True
        if self.SK:
            personarray_msg = PersonArray()
            personarray_msg.header = Header()
            for per_index,persons in enumerate(self.SK):
                if firstskel:
                    skelcolor = self.mainskelcolor
                #     skeltext = '0'

                    firstskel = False
                else:
                    # rospy.logwarn('tanin')
                    skelcolor = mycolor(per_index)#self.otherskelscolor
            # for a,aPoint in enumerate(self.SK[0]):
                # rospy.logdebug(persons)
                thisPerson = Person()
                for a,aPoint in enumerate(persons):

                         #skeltext = '?'
                    skeltext = str(a)
                    thisJoint = Joint()
                    thisJoint.joint_num = a
                    thisJoint.confidence = aPoint.confidence
                    thisJoint.roi.x_offset  = int(aPoint.x-self.markersize)
                    thisJoint.roi.y_offset  = int(aPoint.y-self.markersize)
                    thisJoint.roi.width     = int(aPoint.x+self.markersize)
                    thisJoint.roi.height    = int(aPoint.y+self.markersize)
                    thisPerson.joints.append(thisJoint)
                    # skelcolor = self.mainskelcolor
                    if aPoint.confidence > self.confidence_treshold:
                        cv2.rectangle(self.bg,(int(aPoint.x-self.markersize),int(aPoint.y-self.markersize)),(int(aPoint.x+self.markersize),int(aPoint.y+self.markersize)),skelcolor,1)
                        cv2.putText(self.bg,skeltext,(int(aPoint.x),int(aPoint.y)),self.font,0.5,(255,255,255))
                debugnumpeople +=1
                personarray_msg.data.append(thisPerson)

                # rospy.logdebug('drawn a person %d'%debugnumpeople)
            self.rois.publish(personarray_msg)
            self.ipp.publish(self.cvbridge.cv2_to_imgmsg(self.bg,'bgr8'))

if __name__ == '__main__':

    sf = ShowSkel('array')
    sf.run()
