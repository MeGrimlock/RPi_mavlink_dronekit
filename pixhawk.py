#vehicle = connect('/dev/ttyACM0',baud=57600, wait_ready=True)

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import datetime

import urllib2

connected=False

import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='/dev/ttyUSB0,baudrate=57600',help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()

def attribute_callback(self, attr_name, msg):
    print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, msg)
    global armed
    if attr_name=='armed':
        armed=msg
    if attr_name=='mode':
        print "mode change detected"
    if attr_name=='is_armable':
        if msg==True:
            print "Drone is now ARMABLE for GUIDED mode"
        else:
            print "Drone cannot be ARMED for GUIDED mode"
    if attr_name=='gps_0':
            print 'GPS update',msg

def connectDrone():
    global vehicle
    print "connecting to drone..."
    print args.connect
    vehicle=connect('/dev/ttyUSB0', baud=57600, wait_ready=['system_status','gps_0', 'armed', 'mode']) 
    #Add observer for the vehicle's THR_MIN parameter
    vehicle.parameters.add_attribute_listener('armed',attribute_callback)
    vehicle.parameters.add_attribute_listener('gps0.fix_type',attribute_callback)
    vehicle.parameters.add_attribute_listener('system_status.state',attribute_callback)
    return True

def disconnectDrone():
    global vehicle
    #Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()
    return False
    
def checkDrone():
    vehiclestate()

def droneStatus():
    """
    The status has a state property with one of the following values:

    UNINIT: Uninitialized system, state is unknown.
    BOOT: System is booting up.
    CALIBRATING: System is calibrating and not flight-ready.
    STANDBY: System is grounded and on standby. It can be launched any time.
    ACTIVE: System is active and might be already airborne. Motors are engaged.
    CRITICAL: System is in a non-normal flight mode. It can however still navigate.
    EMERGENCY: System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
    POWEROFF: System just initialized its power-down sequence, will shut down now

    """
    return vehicle.system_status.state

def vehiclestate():
    # vehicle is an instance of the Vehicle class
    print "\tGlobal Location: %s" % vehicle.location.global_frame
    print "\tGlobal Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print "\tLocal Location: %s" % vehicle.location.local_frame    #NED
    #print "\tAttitude: %s" % vehicle.attitude
    #print "\tVelocity: %s" % vehicle.velocity
    print "\tGPS: %s" % vehicle.gps_0
    #print "\tGroundspeed: %s" % vehicle.groundspeed
    #print "\tAirspeed: %s" % vehicle.airspeed
    #print "Gimbal status: %s" % vehicle.gimbal
    print "\tBattery: %s" % vehicle.battery
    print "\tEKF OK?: %s" % vehicle.ekf_ok
    print "\tLast Heartbeat: %s" % vehicle.last_heartbeat
    #print "\tRangefinder: %s" % vehicle.rangefinder
    #print "\tRangefinder distance: %s" % vehicle.rangefinder.distance
    #print "\tRangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "\tHeading: %s" % vehicle.heading
    print "\tIs Armable?: %s" % vehicle.is_armable
    print "\tSystem status: %s" % vehicle.system_status.state
    print "\tMode: %s" % vehicle.mode.name    # settable
    print "\tArmed: %s" % vehicle.armed    # settable

    #print "\tRead vehicle param 'THR_MIN': %s" % vehicle.parameters['THR_MIN']
    #print "\tRead vehicle param 'THR_MAX': %s" % vehicle.parameters['THR_MAX']

def funcion_conectar():
    global vehicle
    global connected
    global pause_script
    
    print "connecting to drone..."
    while connected==False:
        try:
            #vehicle=connect(args.connect, wait_ready=['system_status','gps_0', 'armed', 'mode'])
            vehicle=connect('/dev/ttyACM0', baud=115200, wait_ready=['system_status','gps_0', 'armed', 'mode'])
            connected=True
        except Exception:
            print "unable to connect...retrying"
            time.sleep(0.5)        
    vehicle.add_attribute_listener('armed',attribute_callback)
    vehicle.add_attribute_listener('mode',attribute_callback)
    vehicle.parameters.add_attribute_listener('gps0.fix_type',attribute_callback)
    vehicle.parameters.add_attribute_listener('system_status.state',attribute_callback)
    #vehicle.add_message_listener('HEARTBEAT',beep)

#For testing purposes, you can import this into your code using: import pixhawk as px
#   then you cna call px.connectDrone() and upon successfull connection work with the rest of the functions such as checkDrone()

connectDrone()
while True:
    checkDrone()
    paquete = 'attitude:{'+str(vehicle.attitude)+'},{location:{'+str(vehicle.location.global_frame)+'}'
    urllib2.urlopen("http://52.43.84.1:1880/put?"+paquete).read()
    time.sleep(5)
