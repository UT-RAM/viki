#!/usr/bin/env python
__author__ = 'Original by Antonio // Adapted by Han Wopereis'

"""
This executable launches a node that receives data from the MVN-Studio-Streamer. 
MVN-studio is the software of XSENS. It has built-in streaming capability. Just enable this streaming to start streaming over the network.
This executable is only tested with XSENS MTx-sensors and only for the upper body.
The communication protocol is UDP.

NOTE THAT: THIS NODE IS NOT PUBLISHING ALL DATA. ALSO THE DATA IS NOT PUBLISHED RAW.
SOME PROCESSING IS DONE IN THIS NODE, TO REDUCE THE AMOUNT OF PUBLISHERS AND DATA FLOW.


================================================================================

The data transferring can be started as follows:

--> Streamer <--

1. Enable the network streaming capabilities in MVN-studio
2. Enter this computer's IP-adress (of the receiving part).
3. Stream on the common port. 

--> Receiver <--

3. The receiver is started and opens a socket as well.
4. This socket is connected to the same port.

--> Both <--

6. The receiver-side keeps on listening for messages and decodes whenever receiving.

Note that: If there is no data streamed over the network, this node keeps listening until there is.

================================================================================

Start the receiver node by:
>> rosrun xsens_network_receiver xsens_network_receiver.py xsens_port:=25114

(last thing is optional)
"""

# System imports
import socket
from struct import unpack

# Ros imports
import rospy
from xsens_network_receiver.msg import Xsens_raw


# Define parameters

XSENS_STREAM_PORT = rospy.get_param('xsens_port',25114)
BUFSIZE = 720
SEGMENT_DATA_LENGTH = 32
TIMESTEP = 100
MINLEN = 5

# Define sensor-numbers and according names
PELVIS 			= 1
L5 				= 2
L3 				= 3
T12 			= 4
T8 				= 5
NECK 			= 6
HEAD 			= 7
RIGHT_SHOULDER 	= 8
RIGHT_UPPER_ARM = 9
RIGHT_FOREARM 	= 10
RIGHT_HAND 		= 11
LEFT_SHOULDER	= 12
LEFT_UPPER_ARM	= 13
LEFT_FOREARM	= 14
LEFT_HAND		= 15
RIGHT_FOOT      = 18 

# Define the class segment that can be called for each sensor.
class Segment:
    x = 0.0
    y = 0.0
    z = 0.0
    re = 0.0
    i = 0.0
    j = 0.0
    k = 0.0

def XsensDataReceiver():

	# Create Segment objects
    pelvis			= Segment()
    l5				= Segment()
    l3				= Segment()
    t12				= Segment()
    t8				= Segment()
    neck            = Segment()
    head            = Segment()
    rightShoulder 	= Segment()
    rightUpperArm 	= Segment()
    rightForeArm 	= Segment()
    rightHand 		= Segment()
    leftShoulder	= Segment()
    leftUpperArm	= Segment()
    leftForeArm		= Segment()
    leftHand		= Segment()
    rightFoot       = Segment()
    
  	# Create segments dictionary for easy refering.
    segmentsDict = {PELVIS : pelvis,
                    L5 : l5,
                    L3 : l3,
                    T12 : t12,
                    T8 : t8,
                    NECK : neck,
                    HEAD : head,                    
                    RIGHT_SHOULDER : rightShoulder,
                    RIGHT_UPPER_ARM : rightUpperArm,
                    RIGHT_FOREARM : rightForeArm,
                    RIGHT_HAND : rightHand,
                    LEFT_SHOULDER : leftShoulder,
                    LEFT_UPPER_ARM : leftUpperArm,
                    LEFT_FOREARM : leftForeArm,
                    LEFT_HAND : leftHand,
                    RIGHT_FOOT : rightFoot,  				
    }
    
    # Create segments dictionary with only used segments.
    usedSegmentsDict = {PELVIS : pelvis,
                    RIGHT_HAND : rightHand,
                    LEFT_HAND : leftHand,  				
    }

	# Init ROS
    rospy.init_node('Xsens_network_receiver')
	
	# Init publishers
    pub_right_hand = rospy.Publisher('/Xsens_network_receiver/RightHand', Xsens_raw, queue_size=10)
    pub_left_hand = rospy.Publisher('/Xsens_network_receiver/LeftHand', Xsens_raw, queue_size=10)	
	
    print "===== Initializing Xsens_network_receiver ====="
    print ""
    print "Xsens_node started."
    print "Publisher right-hand started: ", pub_right_hand
    print "Publisher left-hand started: ", pub_left_hand
    print ""
    print "-----------------------------"
    print "Initializing socket: "
    print ""	
		
	# Init socket to receive XSENS streaming
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', XSENS_STREAM_PORT))

    # Init used variables
    oldTime = 0
    msgLeft = Xsens_raw()
    msgRight = Xsens_raw()
    
    print "Socket created: ", s

    print ""
    print "======================================="
    print ""
    print "Xsens-receiver --> Online"
    print ""
    print "---------------------------------------"
    
    # Main loop
    while not rospy.is_shutdown():
        # Await incoming data.
        data = s.recvfrom(BUFSIZE)[0]
        
        if VERBOSE:  
            print("Message of XSENS received.")
      
		# Check message type is correct
        if int(data[4]) == 0 and int(data[5]) == 2:

			# Get time of received samples
            time= unpack('>i', ''.join(data[12:16]))[0]
			
            if VERBOSE:
			    print "Time of message: ", time
			
			# Handle XSENS restart
            if time < TIMESTEP:
                oldTime= 0
                if VERBOSE:
                    print ""
                    print "----------------------"
                    print "Detected XSENS restart"
                    print "----------------------"
                    print ""

			# Drop data if already received and apply subsampling
            if time >= oldTime + TIMESTEP:
                
				# Strip header
                data = data[24:]
                if VERBOSE: 
                    print ""  
                    print "Data: ", data
                    print ""
                    
				# Extract segments data
                splitData = [data[i:i+SEGMENT_DATA_LENGTH] for i in range(0, len(data), SEGMENT_DATA_LENGTH)]

				# Process segments
                for segmentData in splitData:
                    idSeg= unpack('>i', ''.join(segmentData[:4]))[0]
                    if VERBOSE:
                        print "Unpacking segment with ID:", idSeg
                        
                    # Check if necessary to unpack.
                    if idSeg in usedSegmentsDict:
                        if VERBOSE:
                            print "Id recognized."
                            print ""
                             
						# Populate segment fields. Incoming data format is big endian float
                        segmentsDict[idSeg].x= unpack('>f', ''.join(segmentData[4:8]))[0]
                        segmentsDict[idSeg].y= unpack('>f', ''.join(segmentData[8:12]))[0]
                        segmentsDict[idSeg].z = unpack('>f', ''.join(segmentData[12:16]))[0]

                        segmentsDict[idSeg].re= unpack('>f', ''.join(segmentData[16:20]))[0]
                        segmentsDict[idSeg].i= unpack('>f', ''.join(segmentData[20:24]))[0]
                        segmentsDict[idSeg].j= unpack('>f', ''.join(segmentData[24:28]))[0]
                        segmentsDict[idSeg].k= unpack('>f', ''.join(segmentData[28:]))[0]

                        if VERBOSE:
                            print "Unpacked: ", segmentsDict[idSeg]
                        
						# Normalize segment position to pelvis									    
                        if (idSeg != PELVIS):		
                        
                            if VERBOSE:
                                print ""
                                print "Normalizing rotation and height with pelvis."
                            
                            # Normalize the value to the pelvis.
                            vec = (segmentsDict[idSeg].x, segmentsDict[idSeg].y, segmentsDict[idSeg].z)
                            q= (segmentsDict[PELVIS].re, segmentsDict[PELVIS].i, segmentsDict[PELVIS].j, segmentsDict[PELVIS].k)
                            rotatedByPelvis = qv_mult(q, vec)	
                            segmentsDict[idSeg].x = rotatedByPelvis[0]
                            segmentsDict[idSeg].y = rotatedByPelvis[1]
                            segmentsDict[idSeg].z = rotatedByPelvis[2]
                            

                            # Normalize pelvis height as 0.
                            segmentsDict[idSeg].z = segmentsDict[idSeg].z - segmentsDict[PELVIS].z

                            if VERBOSE:
                                print "Result: ", segmentsDict[idSeg]
                                print "==================================="
                                print ""
                                
						# Create the message and send if the message is left hand or right hand
                        if (idSeg == LEFT_HAND):
                            msgLeft.name = "LEFT_HAND"
                            msgLeft.x  = segmentsDict[idSeg].x
                            msgLeft.y  = segmentsDict[idSeg].y
                            msgLeft.z  = segmentsDict[idSeg].z
                            msgLeft.re = segmentsDict[idSeg].re
                            msgLeft.i  = segmentsDict[idSeg].i
                            msgLeft.j  = segmentsDict[idSeg].j
                            msgLeft.k  = segmentsDict[idSeg].k
                            if SIMPLE_VERBOSE or VERBOSE:
                                print "Publish left : " + str(msgLeft.x) + "\t" + str(msgLeft.y) + "\t" + str(msgLeft.z) + "\t" + str(msgLeft.re) + "\t" + str(msgLeft.i) + "\t" + str(msgLeft.j) + "\t" + str(msgLeft.k)
                            pub_left_hand.publish(msgLeft)

                        elif (idSeg == RIGHT_HAND):
                            msgRight.name = "RIGHT_HAND"
                            msgRight.x  = segmentsDict[idSeg].x
                            msgRight.y  = segmentsDict[idSeg].y
                            msgRight.z  = segmentsDict[idSeg].z
                            msgRight.re = segmentsDict[idSeg].re
                            msgRight.i  = segmentsDict[idSeg].i
                            msgRight.j  = segmentsDict[idSeg].j
                            msgRight.k  = segmentsDict[idSeg].k
                            
                            if SIMPLE_VERBOSE or VERBOSE:
                                print "Publish right: " + str(msgRight.x) + "\t" + str(msgRight.y) + "\t" + str(msgRight.z) + "\t" + str(msgRight.re) + "\t" + str(msgRight.i) + "\t" + str(msgRight.j) + "\t" + str(msgRight.k)
                            pub_right_hand.publish(msgRight)
                    else:
                        if VERBOSE:
                            print "Id-seg not recognized in the used segments list."
                            print "------------------------------------------------"

            else:
                if VERBOSE:
                    print "Data sample is not of the next timestep."
                    print ""
        else: 
            if VERBOSE:
                print ""
                print "Message from XSENS is not of the correct type. (data[4,5] != 02)"		
                print "Probably wrong type of Xsens data encoding."		
    s.close()




# Define extra global functionality
def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q_conjugate(q1), q2), q1)[1:]


# Main function
if __name__ == '__main__':

    SIMPLE_VERBOSE = 1
    VERBOSE = 0
    
    # Main loop is in initializer.
    XsensDataReceiver()






