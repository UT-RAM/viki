#!/usr/bin/python

"""
This Python executable is part of the ram ROS package. Its function is to automatically connect to multiple drones. The application is designed to work under Ubuntu Linux (and just that).
For documentation, check the Github repository. http://github.com/ceesietopc/ram_ba_package
"""

# System imports
import sys
import json
import yaml
import dbus                             # communicate with Network manager over DBUS
import time
import telnetlib                        # Telnet to drones for change of configuration
from gi.repository import Gtk, GObject  # Glade interface
import threading
import subprocess
import multiprocessing
import os
import xmlrpclib
import socket

# ROS related imports
import rospy
import rospkg
import rosnode
import rosgraph

# Configuration parameters. Change before use!
SSID = "RAM_drones"                     # SSID of the dedicated network for the drones
SSID_SLUG = "ardrone"                   # Slug to scan SSIDs for, for drone identification
wlan_interface = "wlan0"                # Used interface for scanning for drones

# Initial values, empty lists, etc.
device_path = None
connection_path = None
settings_path = None
controllerProcesses = []
connectors = 0

# Implementation of multithreading with callback on exit. Important to maintain an up to date representation of the running controllers.
class controllerThread(threading.Thread):
    def __init__(self, ssid, onExit, popenArgs):
        self.stdout = None
        self.stderr = None
        threading.Thread.__init__(self)
        self.daemon = True
        self.ssid = ssid
        self.onExit = onExit
        self.popenArgs = popenArgs

    def run(self):
        p = subprocess.Popen(self.popenArgs,
                             shell=False)
        p.wait()
        self.onExit(self.ssid)

def importDrones(buttonPressed):
    # If import button is pressed, read drones and write them to list

    # Start of with getting a list of drone objects from a JSON file
    json_data = open(package_location+"/py/drones.json").read()
    data = json.loads(json_data)

    # IP Address can change everytime. SSID, Prefix and trackable_id should remain the same. Just read those values. The other columns in the list are empty
    store = builder.get_object("liststore1")
    store.clear()
    for drone in data:
        store.append([drone['ssid'], "",drone['prefix'], drone['trackable_id'], "", ""])
    # Make sure to update the optiTrack configuration when new drones are added
    optiTrackUpdate()

def exportDrones(buttonPressed):
    # If export drones is pressed, write the drones to a JSON list
    
    drones = [];
    store = builder.get_object("liststore1")
    if len(store) < 1:
        return False

    for drone in store:
        drones.append({'ssid': drone[0], 'prefix': drone[2], 'trackable_id': drone[3]})
    with open(package_location+'/py/drones.json','w') as outfile:
            json.dump(drones,outfile)

def getNextIP():
    # Returns the next available IP list to assign to a drone. Loops through list and checks for largest value, than adds 1.
    ip = 9;
    inuse = True
    store = builder.get_object("liststore1")

    while inuse:
        inuse = False
        for drone in store:
            if drone[1] == "192.168.2."+str(ip):
                inuse = True
                ip += 1
                continue
    return ip

def assignIP(buttonPressed):
    # Loop through all available drones and change their configuration through telnet.

    # Loop through store
    store = builder.get_object("liststore1")
    for drone in store:
        # If no IP yet AND ad-hoc available
        if (drone[1] == "") and (drone[4] == "Yes"):
            # Get the next available IP
            ip = str(getNextIP())

            # Connect to the the drone
            print "Connecting!"
            droneConnect(drone[0])

            # Reconfigure over Telnet
            tn = telnetlib.Telnet("192.168.1.1")
            tn.read_some()
            tn.write("rm /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"killall udhcpd\" > /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"ifconfig ath0 down\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"iwconfig ath0 mode managed essid "+SSID+"\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"ifconfig ath0 192.168.2."+ip+" netmask 255.255.255.0 up\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("chmod +x /data/wifi.sh\n");
            tn.read_some()
            tn.write("sh /data/wifi.sh\n")
            # IMPORTANT At this point, we lose connectivity

            disconnect()

            # Update information in store 
            drone[1] = "192.168.2."+ip
            drone[4] = "No"

def findNetworks():
    # Scan for drones.
    drones = []
    global device_path

    # Enable Wireless. If Wireless is already enabled, this does nothing.
    was_wifi_enabled = manager_props.Get("org.freedesktop.NetworkManager",
                                         "WirelessEnabled")
    if not was_wifi_enabled:
        print "Enabling WiFi and sleeping for 10 seconds ..."
        manager_props.Set("org.freedesktop.NetworkManager", "WirelessEnabled",
                          True)
        # Give the WiFi adapter some time to scan for APs. This is absolutely
        # the wrong way to do it, and the program should listen for
        # AccessPointAdded() signals, but it will do.
        time.sleep(10)

    accesspoints_paths_list = device.GetAccessPoints()

    # Identify our access point. We do this by comparing our desired SSID
    # to the SSID reported by the AP.
    our_ap_path = None
    for ap_path in accesspoints_paths_list:
        ap_props = dbus.Interface(
            bus.get_object("org.freedesktop.NetworkManager", ap_path),
            "org.freedesktop.DBus.Properties")
        ap_ssid = ap_props.Get("org.freedesktop.NetworkManager.AccessPoint",
                               "Ssid")
        # Returned SSID is a list of ASCII values. Let's convert it to a proper
        # string.
        str_ap_ssid = "".join(chr(i) for i in ap_ssid)
        print ap_path, ": SSID =", str_ap_ssid
        # If the SSID matches, store drone
        if SSID_SLUG in str_ap_ssid:
            drones.append({'ap_path': ap_path, 'ssid': str_ap_ssid})
    return drones

def droneConnect(ssid):
    # Connect to a drone, based on SSID
    global device_path, connection_path, settings_path

    # Connect to the device's Wireless interface and obtain list of access
    # points.
    device = dbus.Interface(bus.get_object("org.freedesktop.NetworkManager",
                                           device_path),
                            "org.freedesktop.NetworkManager.Device.Wireless")
    accesspoints_paths_list = device.GetAccessPoints()

    # Identify our access point. We do this by comparing our desired SSID
    # to the SSID reported by the AP.
    our_ap_path = None
    for ap_path in accesspoints_paths_list:
        ap_props = dbus.Interface(
            bus.get_object("org.freedesktop.NetworkManager", ap_path),
            "org.freedesktop.DBus.Properties")
        ap_ssid = ap_props.Get("org.freedesktop.NetworkManager.AccessPoint",
                               "Ssid")
        # Returned SSID is a list of ASCII values. Let's convert it to a proper
        # string.
        str_ap_ssid = "".join(chr(i) for i in ap_ssid)
        print ap_path, ": SSID =", str_ap_ssid
        # Make a foreach something here
        if ssid == str_ap_ssid:
            our_ap_path = ap_path

    # At this point we have all the data we need. Let's prepare our connection
    # parameters so that we can tell the NetworkManager what is the passphrase.
    connection_params = {}

    # Establish the connection.
    settings_path, connection_path = manager.AddAndActivateConnection(
        connection_params, device_path, our_ap_path)
    print "settings_path =", settings_path
    print "connection_path =", connection_path

    # Wait until connection is established. This may take a few seconds.
    NM_ACTIVE_CONNECTION_STATE_ACTIVATED = 2
    print """Waiting for connection to reach """ \
          """NM_ACTIVE_CONNECTION_STATE_ACTIVATED state ..."""
    connection_props = dbus.Interface(
        bus.get_object("org.freedesktop.NetworkManager", connection_path),
        "org.freedesktop.DBus.Properties")
    state = 0
    while True:
        # Loop forever until desired state is detected.
        #
        # A timeout should be implemented here, otherwise the program will
        # get stuck if connection fails.
        #
        # IF PASSWORD IS BAD, NETWORK MANAGER WILL DISPLAY A QUERY DIALOG!
        # This is something that should be avoided, but I don't know how, yet.
        #
        # Also, if connection is disconnected at this point, the Get()
        # method will raise an org.freedesktop.DBus.Error.UnknownMethod
        # exception. This should also be anticipated.
        state = connection_props.Get(
            "org.freedesktop.NetworkManager.Connection.Active", "State")
        if state == NM_ACTIVE_CONNECTION_STATE_ACTIVATED:
            break
        time.sleep(0.001)
    print "Connection established!"

def disconnect():
    # Clean up: disconnect and delete connection settings. If program crashes
    # before this point is reached then connection settings will be stored
    # forever.
    # Some pre-init cleanup feature should be devised to deal with this problem,
    # but this is an issue for another topic.
    global device_path, connection_path, settings_path

    bus = dbus.SystemBus()
    # Obtain handles to manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

    manager.DeactivateConnection(connection_path)
    settings = dbus.Interface(
        bus.get_object("org.freedesktop.NetworkManager", settings_path),
        "org.freedesktop.NetworkManager.Settings.Connection")
    settings.Delete()

def scanDrones(buttonPressed):
    # Scan for GAZEBO running
    gazeboLocation = rosnode.get_api_uri(rosgraph.Master('/rosout'),'/gazebo')    
    print ""
    print "Gazebo is located:" , gazeboLocation
    print ""
    # Find new drones. For each drone found, check if the thing is in the list.

        
    # Function that actually scans for the drones
    drones = findNetworks()
  
            
    # No drones returned and no simulation, error.
    if len(drones) < 1 and gazeboLocation==None:
        warning(window,"Make sure the batteries of the AR.Drones are connected.","No drones or Gazebo found!")
        return None
    else:
        # Add drones we do not know yet to the store.
        store = builder.get_object("liststore1")

        if len(drones) > 0:    
            for newDrone in drones:
                # loop through the ones in the storage and check if we have equal ssid
                found = False
                for drone in store:
                    if newDrone['ssid'] == drone[0]:
                        # SSID is equal. Do not add to list. Add that we found it though!
                        found = True
                        drone[4] = "Yes"
                if found == True:
                    continue
                # New drone found. Ask information and add  to the list
                dialog = askDialog(window, "Please enter a prefix and trackable ID for the new drone.",newDrone['ssid']+" identification")
                if dialog != None:
                    store.append([newDrone['ssid'], "", dialog[0], int(dialog[1]), "Yes", ""])

        if (gazeboLocation!=None):
            print "Found Gazebo"
            # Add simulation drone drones we do not know yet to the store.
            simulateDroneFound = False
            for drone in store:
                if drone[0] == "SimulationDrone":
                    simulateDroneFound = True
            if not simulateDroneFound:
                store.append(["SimulationDrone", "-1", "SimulationDrone", 999, "Yes", "No"])      

    # List changed, ipdate OptiTrack!
    optiTrackUpdate()

def warning(parent, message, title=''):
    # Helper function for posing warnings
    dialogWindow = Gtk.MessageDialog(parent,
                      Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                      Gtk.MessageType.QUESTION,
                      Gtk.ButtonsType.OK,
                      message)
    dialogWindow.set_title(title)
    dialogWindow.show_all()
    response = dialogWindow.run()
    dialogWindow.destroy()
    return response

def confirm(parent, message, title=''):
    # Helper function for posing confirmation window
    dialogWindow = Gtk.MessageDialog(parent,
                      Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                      Gtk.MessageType.QUESTION,
                      Gtk.ButtonsType.OK_CANCEL,
                      message)

    dialogWindow.set_title(title)
    dialogWindow.show_all()
    response = dialogWindow.run()
    dialogWindow.destroy()
    return response

def askIp(parent, message, title=''):
    # Helper function to ask for an IP
    dialogWindow = Gtk.MessageDialog(parent,
                          Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                          Gtk.MessageType.QUESTION,
                          Gtk.ButtonsType.OK_CANCEL,
                          message)

    dialogWindow.set_title(title)
    dialogBox = dialogWindow.get_content_area()

    ipLabel = Gtk.Label()
    ipLabel.set_text("IP Address:")
    dialogBox.pack_start(ipLabel, False, False, 0)

    ipEntry = Gtk.Entry()
    ipEntry.set_size_request(250,0)
    dialogBox.pack_start(ipEntry, False, False, 0)

    dialogWindow.show_all()
    response = dialogWindow.run()
    ip = ipEntry.get_text() 
    dialogWindow.destroy()
    if (response == Gtk.ResponseType.OK):
        return ip
    else:
        return None


def askDialog(parent, message, title=''):
    # Helper function to ask for a prefix and trackable ID
    dialogWindow = Gtk.MessageDialog(parent,
                          Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                          Gtk.MessageType.QUESTION,
                          Gtk.ButtonsType.OK_CANCEL,
                          message)

    dialogWindow.set_title(title)
    dialogBox = dialogWindow.get_content_area()

    nameLabel = Gtk.Label()
    nameLabel.set_text("Prefix / name:")
    dialogBox.pack_start(nameLabel, False, False, 0)

    nameEntry = Gtk.Entry()
    nameEntry.set_size_request(250,0)
    dialogBox.pack_start(nameEntry, False, False, 0)

    trackableLabel = Gtk.Label()
    trackableLabel.set_text("Trackable ID (integer):")
    dialogBox.pack_start(trackableLabel, False, False, 0)

    trackableEntry = Gtk.Entry()
    trackableEntry.set_size_request(250,0)
    dialogBox.pack_start(trackableEntry, False, False, 0)

    dialogWindow.show_all()
    response = dialogWindow.run()
    name = nameEntry.get_text() 
    trackable = trackableEntry.get_text() 
    dialogWindow.destroy()
    if (response == Gtk.ResponseType.OK) and (isinstance(int(trackable), (int))):
        return [name, trackable]
    else:
        return None

def launchController(treeview, row, column):
    # This function should only run when connected to the dedicated network. This can be done through wifi and cable though, so do not do it automatically.
    # Check for active drones, otherwise give warning. This function is activated by double clicking a drone.
    global controllerProcesses

    store = builder.get_object("liststore1")
    if store[row][1] == "":
        warning(window, "Please assign IP's first.","No IP address assigned to this drone.")
        return None

    # warn user that he/she should be connected to the subnet the drones use
    if len(controllerProcesses) < 1:
        response = confirm(window,"Please make sure you are connected to the dedicated network before running a controller.")
        if response != Gtk.ResponseType.OK:
            return None

    # It can be that the ssid is already active, give no warning then
    if store[row][5] == "Yes":
        warning(window, "Controller already active.","A controller was already initiated for this drone.")
        return None
    
    # Update list with information
    store[row][5] = "Yes"

    # Start the new thread
    th = controllerThread(store[row][0], garbageCollection, ["rosrun", "ram", "controller.py", store[row][2], store[row][1], str(int(connectors))])
    th.start()

    # Do bookkeeping
    controllerProcesses.append(store[row][0])

def garbageCollection(ssid):
    # This function is called when a controller process is ended. At that point, the list should reflect that.
    global controllerProcesses
    # Get list of processes and remove this one
    controllerProcesses.remove(ssid) 

    store = builder.get_object("liststore1")
    for drone in store:
        if drone[0] == ssid:
            drone[5] = "No"


def quitCallback(widget, event):
    # This function is called when the window close button is called.
    global controllerProcesses, optiThread

    # We should not close the interface when there are active controllers
    if len(controllerProcesses) > 0:
        warning(window, "Controllers active.","There are active controller processes. Please close them first.")
        return True

    # Close the optitrack thread
    try:
        optiThread.poll()
        if(optiThread.returncode == None):
            optiThread.terminate()
    except (NameError, AttributeError):
        pass

    # Close the joystick thread
    try:
        joyThread.poll()
        if(joyThread.returncode == None):
            joyThread.terminate()
    except (NameError, AttributeError):
        pass

    # Quit the application and return to terminal
    Gtk.main_quit(widget, event)

def btnClear(buttonPressed):
    # Clear the list. Drones that have active controllers are not cleared.
    store = builder.get_object("liststore1")
    for (i, drone) in enumerate(store):
        if(drone[5] != "Yes"):
            store.remove(drone.iter)

def rightClick(treeview, event):
    # When a right click is performed, give the user the ability to change the IP manually
    if event.button == 3: # right click
        store = builder.get_object("liststore1")
        path, col, x, y = treeview.get_path_at_pos(int(event.x), int(event.y))
        ip = askIp(window, "Enter custom IP","")
        if ip != None:
            treeiter = store.get_iter(path)
            store.set_value(treeiter, 1, ip)
            store.set_value(treeiter, 4, "No")  # After setting the IP, it is assumed the thing is not Ad-hoc availabe

def optiTrackUpdate():
    # Function to update the optitrack configuration
    
    # Close current thread
    global optiThread, connectors
    try:
        optiThread.poll()
        if(optiThread.returncode == None):
            optiThread.terminate()
    except (NameError, AttributeError):
        pass

    # Get location to store the config
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    store = builder.get_object("liststore1")
    
    # Write output by hand. Keep track of the highest trackable ID, so mass connectors can be added afterwards.
    maxvalue = 0
    with open(package_location+'/launch/optitrack_instance.yaml', 'w') as outfile:
        outfile.write("rigid_bodies:\n")
        for drone in store:
            outfile.write("    '"+str(drone[3])+"':\n")
            outfile.write("        pose: "+drone[2]+"/unfiltered_pose\n")

            if drone[3] > maxvalue:
                maxvalue = drone[3]

        # ASSUMPTION: All mass connectors have subsequent trackable IDs. The first mass connector has is the highest ID in the drone list + 1.
        connector = 0
        while connector != connectors:
            connector += 1
            outfile.write("    '"+str(maxvalue + connector)+"':\n")
            outfile.write("        pose: massConnector"+str(connector)+"/unfiltered_pose\n")
    print "Launching optitrack:"
    # Start optitrack again
    optiThread = subprocess.Popen(["roslaunch", "ram", "optitrack_instance.launch"])
    print "Optitrack launched!"

def adjMassConnectors(adj):
    # This function is used when the amount of mass connectors is changed. It initiates an update of the OptiTrack configuration
    global connectors
    connectors = adj.get_value()
    optiTrackUpdate()

def btnSimControl(btn):
    # If the "batch edit setpoints" button is clicked, a separate executable is opened, with the names of the drones as arguments.
    args = ["rosrun", "ram", "simControl.py"];
    store = builder.get_object("liststore1")
    for drone in store:
        args.append(drone[2])
    p = subprocess.Popen(args, shell=False)

# Main function
if __name__ == "__main__":
    # Check for the presence of a roscore by checking the ROS_MASTER_URI 
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('interface_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    # We update the interface directly through the python code, use threads
    GObject.threads_init()

    # Enable joystick by running joy node. Every controller has a separate interpreter
    joyThread = subprocess.Popen(["rosrun", "joy", "joy_node"])

    # Get the location of the glade file and use it for the interface
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    gladefile = package_location+"/py/interface.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    # Connect all signals from the Glade file
    handlers = {
        "onDeleteWindow": quitCallback,
        "btnImport": importDrones,
        "btnScan": scanDrones,
        "btnExport": exportDrones,
        "btnAssign": assignIP,
        "btnSimControl": btnSimControl,
        "launchController": launchController,
        "btnClear": btnClear,
        "rightClick": rightClick,
        "adjMassConnectors": adjMassConnectors
    }
    builder.connect_signals(handlers)

    # Initiate the system bus 
    bus = dbus.SystemBus()
    # Obtain handles to network manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

    
    # Get path to the 'wlan0' device. 
    device_path = manager.GetDeviceByIpIface(wlan_interface)
    print "wlan path: ", device_path

    # Connect to the device's Wireless interface and obtain list of AP's
    device = dbus.Interface(bus.get_object("org.freedesktop.NetworkManager",
                                           device_path),
                            "org.freedesktop.NetworkManager.Device.Wireless")

    # Show window and start main loop
    window = builder.get_object("window1")
    window.show_all()
    Gtk.main()
