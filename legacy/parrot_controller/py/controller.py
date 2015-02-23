#!/usr/bin/env python

"""
This Python executable is part of the ram ROS package. Its function is to handle the position control of a single quadcopter. The application is designed to work under Ubuntu Linux (and just that).
For documentation, check the Github repository. http://github.com/ceesietopc/ram_ba_package
"""

# System imports
import sys
import time
from gi.repository import Gtk, cairo, Pango, PangoCairo, GObject, Gdk   # Interface design using Glade
import threading
import os
import xmlrpclib
import socket
import subprocess
import math

# ROS related imports
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf.transformations
import rospkg
from ram.msg import nonlinearity
from ram.msg import gains


# The RosConnector class is responsible for all communication with the ROS infrastructure. All nodes and subscribers, including callback, can be found in this class.
class RosConnector:
    def __init__(self, prefix, ip):
        # These subscribers are self-explanatory: they send basic commands to the drone or controller
        self.pubTakeoff = rospy.Publisher("/"+prefix+"/ardrone/takeoff", std_msgs.msg.Empty, queue_size=1)
        self.pubLand = rospy.Publisher("/"+prefix+"/ardrone/land", std_msgs.msg.Empty, queue_size=1)
        self.pubReset = rospy.Publisher("/"+prefix+"/ardrone/reset", std_msgs.msg.Empty, queue_size=1)
        self.pubSetpoint = rospy.Publisher("/"+prefix+"/setpoint", geometry_msgs.msg.Pose)
        self.pubNonLinear = rospy.Publisher("/"+prefix+"/nonlinearity", nonlinearity)
        self.pubGains = rospy.Publisher("/"+prefix+"/gains", gains)

        # Since we are able to change the source of the commands (controller or joystick), the connector listens for these messages and republishes the one that is currently configured to be the output.
        self.subJoystick = rospy.Subscriber("/"+prefix+"/cmd_vel_joy", geometry_msgs.msg.Twist, self.joyCB)
        self.subController = rospy.Subscriber("/"+prefix+"/cmd_vel_controller", geometry_msgs.msg.Twist, self.controllerCB)
        self.pubCmd = rospy.Publisher("/"+prefix+"/cmd_vel", geometry_msgs.msg.Twist)
        
        # To enable joystick control from the joystick itself, listen to the raw joystick messages as well.
        self.subJoyRaw = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.joyRawCB, queue_size=1) 

        # For live setpoint updates in the user interface, listen for setpoints as well.
        self.subSetpoint = rospy.Subscriber("/"+prefix+"/setpoint", geometry_msgs.msg.Pose, self.setpointCB)
        self.setpoint = geometry_msgs.msg.Pose()

        # Run the actual controller (C++ executable)
        self.pr = subprocess.Popen(["roslaunch","ram", "controller_module.launch", "prefix:="+prefix, "ip:="+ip])

        # Prepare dictionaries for subscribers and positions of the mass connectors.
        self.massConnectorSubscriberDict = {}
        self.massConnectorPositionDict = {}

    def joyCB(self, msg):
        # If the joystick checkbox is ticked, publish the cmds from the cmd_vel_joy to cmd_vel
        global publishJoystick
        if publishJoystick:
            self.pubCmd.publish(msg)

    def setpointCB(self, msg):
        # As a setpoint is received, change the member of this class.
        # Since this has to do with the UI, enter threads first.
        Gdk.threads_enter()
        self.setpoint = msg
        Gdk.threads_leave()

    def joyRawCB(self, msg):
        # If a certain joystick button is pushed AND the window is active, enable joystick control.
        # Since this has to do with the UI, enter threads first. 
        Gdk.threads_enter()
        if window.is_active():
            if msg.buttons[3] == 1:         # Change button to 3 for PS3, 8 for Joystick btn 9
                toggleJoystickCheckbox()
                time.sleep(0.3)             # Sleep a moment to make sure repetitive commands are not misinterpreted
        Gdk.threads_leave()

    def controllerCB(self, msg):
        # If a command from the controller is received, republish to cmd_vel, unless publish zero, publish joystick or publish nothing are ticked.
        global publishJoystick
        global publishZero
        global publishNothing

        if not publishJoystick and not publishZero and not publishNothing:
            # Publish controller output
            self.pubCmd.publish(msg)
        if publishNothing and not publishJoystick:
            # Publish a non hover, zero velocity/attitude message 
            msg = geometry_msgs.msg.Twist()
            msg.angular.x = 1;
            msg.angular.y = 1;
            self.pubCmd.publish(msg)
        if publishZero and not publishNothing and not publishJoystick:
            # Publish an empty message (referring to "enable hovermode")
            self.pubCmd.publish(geometry_msgs.msg.Twist())
    
    def clean(self):
        # On closing the application, terminate the running C++ controller.
        print "Cleaning!"
        try:
            self.pr.terminate()
        except (OSError, AttributeError):
            # can't kill a dead/non existent proc
            pass

    def takeOff(self):
        # Send take-off message
        msg = std_msgs.msg.Empty()
        self.pubTakeoff.publish(msg)

    def land(self):
        # Send land message
        msg = std_msgs.msg.Empty()
        self.pubLand.publish(msg)

    def reset(self):
        # Send reset message
        msg = std_msgs.msg.Empty()
        self.pubReset.publish(msg)

    def setSetpoint(self, x, y, z, yaw):
        # Function to set the setpoint of the drone. 
        msg = geometry_msgs.msg.Pose()
        msg.position.x = x
        msg.position.y = y;
        msg.position.z = z;

        # Transformation from yaw to quaternion. All other attitudes are not important.
        quaternion = tf.transformations.quaternion_from_euler(0, 0, (yaw*2*math.pi)/(360))
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pubSetpoint.publish(msg)

    def addMassListener(self, prefix):
        # Add a subscriber for a mass connector to the dictionary, that can be found by a prefix (e.g. massConnector1). All these subscribers share a single callback.
        self.massConnectorSubscriberDict[prefix] = rospy.Subscriber("/"+prefix+"/unfiltered_pose", geometry_msgs.msg.Pose, self.massCB, callback_args=prefix)

    def massCB(self, pose, prefix):
        # Callback for the mass connectors. Save the received pose in the dictionary, identified by the prefix of the mass connector.
        self.massConnectorPositionDict[prefix] = pose

    def getMassConnectorLocation(self, prefix):
        # Return the location of the mass connector from the dictionary.
        return self.massConnectorPositionDict[prefix]

    def setNonLinear(self, x, y, z, xoff, yoff):
        # Set the nonlinearity of the controller, by sending a nonlinearity() message.
        msg = nonlinearity()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.xoff = xoff
        msg.yoff = yoff
        self.pubNonLinear.publish(msg)

    def setGains(self, p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, i_enabled, v_enabled):
        # Set the gains, by sending a gains() message.
        msg = gains()
        msg.p_z = p_z
        msg.d_z = d_z
        msg.p_rot = p_rot
        msg.d_rot = d_rot
        msg.p_trans = p_trans
        msg.d_trans = d_trans
        msg.i_action = i_action
        msg.v_damping = v_damping
        msg.i_enabled = i_enabled
        msg.v_enabled = v_enabled
        self.pubGains.publish(msg)
# End RosConnector

def btnClose(widget, event):
    # This function is executed when the close button of the window is pressed.
    
    # Clean up controller.
    ros.clean()

    # Close the joy node.
    try:
        joyThread.poll()
        if(joyThread.returncode == None):
            joyThread.terminate()
    except (NameError, AttributeError):
        pass

    # Quit and return to terminal.
    Gtk.main_quit(widget, event)

def btnTakeOff(b):
    # Take-off button listener
    ros.takeOff()

def btnLand(b):
    # Land button listener
    ros.land()

def btnReset(b):
    # Reset button listener
    ros.reset()

def boxJoystick(box):
    # Called if the box for joystick control is ticked or unticked. 
    global publishJoystick
    publishJoystick = box.get_active()

def boxSetpoint(box):
    # Called if the box for the setpoint publishing is ticked or unticked.
    global publishSetpoint
    publishSetpoint = box.get_active()
    if publishSetpoint == True:
        setSetpoint()

def boxZero(box):
    # Called if the box for the hover publishing is ticked or unticked.
    global publishZero
    publishZero = box.get_active()

def boxNothing(box):
    # Called if the box for the "nothing" publishing is ticked or unticked.
    global publishNothing
    publishNothing = box.get_active()

# Update setpoint whenever the scaleStores change value.
def scaleX(scale):
    setSetpoint()

def scaleY(scale):
    setSetpoint()

def scaleZ(scale):
    setSetpoint()

def scaleYaw(scale):
    setSetpoint()

def setSetpoint():
    # Get setpoint from sliders and send to controller IF we want to do that directly
    global publishSetpoint
    setx = builder.get_object("scalestoreX").get_value()
    sety = builder.get_object("scalestoreY").get_value()
    setz = builder.get_object("scalestoreZ").get_value()
    setyaw = builder.get_object("scalestoreYaw").get_value()
    if publishSetpoint:
        ros.setSetpoint(setx, sety, setz, setyaw)

def cmbMassConnectors(cmb):
    pass

def btnSetpointFromMassConnector(btn):
    # Prepare the picking up from a mass connector
    
    # Get the pose of the mass connector selected in the list
    active = builder.get_object("cmbMassConnectors").get_active()
    text = builder.get_object("listStoreMassConnectors")[active][0]
    pose = ros.getMassConnectorLocation(text)
    
    # Go to a position just in front of the mass connector.
    builder.get_object("scalestoreX").set_value(pose.position.x+0.3)
    builder.get_object("scalestoreY").set_value(pose.position.y)
    builder.get_object("scalestoreZ").set_value(pose.position.z+0.27)
    builder.get_object("scalestoreYaw").set_value(0)

def btnHookForward(btn):
    # Go forward (and slightly up) to actually pick-up the mass connector.
    builder.get_object("scalestoreX").set_value(builder.get_object("scalestoreX").get_value()-0.45)
    builder.get_object("scalestoreZ").set_value(builder.get_object("scalestoreZ").get_value()+0.05)

def btnSetNonLinear(btn):
    # Read nonlinearity from the sliders, send to ROS connector
    x = builder.get_object("scalestoreNonLinX").get_value()
    y = builder.get_object("scalestoreNonLinY").get_value()
    z = builder.get_object("scalestoreNonLinZ").get_value()
    offx = builder.get_object("scalestoreOffsetX").get_value()
    offy = builder.get_object("scalestoreOffsetY").get_value()
    ros.setNonLinear(x,y,z,offx,offy)

def btnGains(btn):
    # Read gains from sliders, send to ROS connector
    p_z = builder.get_object("scalestoreGainPZ").get_value()
    d_z = builder.get_object("scalestoreGainDZ").get_value()
    p_rot = builder.get_object("scalestoreGainPRot").get_value()
    d_rot = builder.get_object("scalestoreGainDRot").get_value()
    p_trans = builder.get_object("scalestoreGainPTrans").get_value()
    d_trans = builder.get_object("scalestoreGainDTrans").get_value()
    i_action = builder.get_object("scalestoreGainI").get_value()
    v_damping = builder.get_object("scalestoreGainVel").get_value()
    i_enabled = builder.get_object("boxIAction").get_active()
    v_enabled = builder.get_object("boxVelDamping").get_active()
    ros.setGains(p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, i_enabled, v_enabled)

def toggleJoystickCheckbox():
    # Toggle if joystick control should be enabled or disabled.
    currentstate = builder.get_object("boxJoystick").get_active()
    if currentstate:
        builder.get_object("boxJoystick").set_active(False)
    else:
        builder.get_object("boxJoystick").set_active(True)

def draw(w, d):
    pass

def readSetpointFromConnector():
    # If the live update box is ticked, update the sliders from the current ros setpoint in the Connector. This function is executed every 250ms.
    if builder.get_object("boxLive").get_active():
        builder.get_object("scaleX").set_value(ros.setpoint.position.x)
        builder.get_object("scaleY").set_value(ros.setpoint.position.y)
        builder.get_object("scaleZ").set_value(ros.setpoint.position.z)
        euler = tf.transformations.euler_from_quaternion([ros.setpoint.orientation.x, ros.setpoint.orientation.y, ros.setpoint.orientation.z, ros.setpoint.orientation.w])
        builder.get_object("scaleYaw").set_value(euler[2])
    return True

# Main function
if __name__ == "__main__":

    # Check for the presence of a roscore by checking the ROS_MASTER_URI 
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('controller_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    # This thing should only be started with an IP address, prefix and number of mass connectors as arguments
    if len(sys.argv) < 4:
        print "Please only initiate controllers from the interface."
        sys.exit()
    else:
        prefix = sys.argv[1]
        ip = sys.argv[2]
        massConnectors = int(sys.argv[3])

    # Initial values
    publishSetpoint = False
    publishJoystick = False
    publishZero = False
    publishNothing = False
    previoustime = 0

    # Initialize node
    rospy.init_node('controller_'+prefix)

    # Execute slider update from setpoint every 250 ms
    GObject.timeout_add(250, readSetpointFromConnector)

    # Threaded application, we want to be able to update the interface from this application directly
    GObject.threads_init()
    Gdk.threads_init()

    # Get glade file for the interface
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    gladefile = package_location+"/py/controller.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    # Init RosConnector for communication to ROS
    ros = RosConnector(prefix, ip)

    # Add mass connectors to list and subscribe to topics of their poses by adding subscribers.
    connector = 0
    store = builder.get_object("listStoreMassConnectors")
    while connector != massConnectors:
        connector = connector + 1
        ros.addMassListener("massConnector"+str(connector))
        store.append(["massConnector"+str(connector)])

    # Connect all buttons from the Glade interface
    handlers = {
        "quit": btnClose,
        "btnTakeOff": btnTakeOff,
        "btnLand": btnLand,
        "btnReset": btnReset,
        "boxJoystick": boxJoystick,
        "boxSetpoint": boxSetpoint,
        "boxZero": boxZero,
        "boxNothing": boxNothing,
        "scaleX": scaleX,
        "scaleY": scaleY,
        "scaleZ": scaleZ,
        "scaleYaw": scaleYaw,
        "draw": draw,
        "cmbMassConnectors": cmbMassConnectors,
        "btnSetpointFromMassConnector": btnSetpointFromMassConnector,
        "btnHookForward": btnHookForward,
        "btnSetNonLinear": btnSetNonLinear,
        "btnGains": btnGains
    }
    builder.connect_signals(handlers)

    # Get window and set title, show window
    window = builder.get_object("window1")
    window.set_title(prefix + " ("+ip+")")
    window.show_all()

    # Run main loop
    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()