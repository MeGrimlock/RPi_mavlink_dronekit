from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import datetime
import pyttsx

#FLAGS de Control:

#Indica si tengo un Dron
connected=False
#Indica si el dron se encuentra armado y en tierra repsectivamente
armed=False
onLand=True
mode=""
#Me indica si uso botones o consola
manual=False
pause_script=False

armDelay=2
reconnectDelay=10
#-------------------------------------------------Raspberry pi GPIO------------------------------------------------
engine_GPIO = pyttsx.init()

try:
    import RPi.GPIO as GPIO
    print "RASPI GPIO detected"
    engine_GPIO.say("RASPI GPIO detected")
    engine_GPIO.runAndWait()

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    #LEDs
    alertLED=31
    armLED=33
    gpsLED=35
    onLED=37
    #buttons
    armBUTTON=40
    landBUTTON=38
    missionBUTTON=36
    print "Setting up GPIO..."
    engine_GPIO.say("Setting up GPIO...")
    engine_GPIO.runAndWait()
    GPIO.setup(alertLED, GPIO.OUT) #Alert (STATUS = CRITICAL or EMERGENCY)
    GPIO.setup(onLED, GPIO.OUT)#On  (connected)
    GPIO.setup(gpsLED, GPIO.OUT)#GPS (3d FIX) 
    GPIO.setup(armLED, GPIO.OUT)#ARM (Drone Armed)
    
    GPIO.setup(armBUTTON,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(landBUTTON,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(missionBUTTON,GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print "Desactivating LEDs..."
    engine_GPIO.say("Desactivating LEDs...")
    engine_GPIO.runAndWait()
    GPIO.output(alertLED,False)
    GPIO.output(armLED,False)
    GPIO.output(gpsLED,False)
    GPIO.output(onLED,False)
    print "GPIO ready!"
    engine_GPIO.say("GPIO ready!")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.runAndWait()

    def ledCycle():
        led=True
        for i in range(0,4):
            GPIO.output(alertLED,led)
            GPIO.output(armLED,led)
            GPIO.output(gpsLED,led)
            GPIO.output(onLED,led)
            led = not led
            time.sleep(2)

    def updateLeds():
        global vehicle
        global armed
        #ALERT
        status=droneStatus()
        if status=="EMERGENCY" or status=="CRITICAL":
            GPIO.output(alertLED,True)
        else:
            GPIO.output(alertLED,False)
        #Connection OK
        if vehicle!=None or connected==True:
            GPIO.output(onLED,True)
        else:
            GPIO.output(onLED,False)
        #GPS
        if int(vehicle.gps_0.fix_type)>=3 and connected==True:
            GPIO.output(gpsLED,True)
        else:
            GPIO.output(gpsLED,False)
        #ARMED
        if vehicle.armed==False and connected==True:
            GPIO.output(armLED,False)
        else:
            GPIO.output(armLED,True)
    manual=False
    print "Testing LEDs"
    engine_GPIO.say(" ")
    engine_GPIO.say("Testing LEDs")
    engine_GPIO.runAndWait()
    ledCycle()
    print "Test complete!"
    engine_GPIO.say("Test complete!")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.say(" ")
    engine_GPIO.runAndWait()
    
except :
    manual=True
    print "NO GPIO detected, running on debug mode"
    
#------------------------------------------------------------------------------------------------------------------
#Set up option parsing to get connection string
import argparse  

parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()

test=False 

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
    if manual==False:
        updateLeds()
    
#Callback method for new messages
lastBeep=-1

def beep(self, name, msg):
    print "beep..."

#------------------------------------------------------------------------------------------------------------------

def connectDrone():
    global vehicle
    print "connecting to drone..."
    vehicle=connect(args.connect, wait_ready=True)    
    #Add observer for the vehicle's THR_MIN parameter
    vehicle.parameters.add_attribute_listener('armed',attribute_callback)
    vehicle.parameters.add_attribute_listener('gps0.fix_type',attribute_callback)
    vehicle.parameters.add_attribute_listener('system_status.state',attribute_callback)
    updateLeds()
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

#------------------------------------------------------------------------------------------------------------------
    
def wait(tiempo):
    time.sleep(tiempo)
    
def armit(intentos=3):
    global vehicle        
    print "Arming motors..."
    # Copter should arm in GUIDED mode
    setmode("STABILIZE")
    #vehicle.armed   = True    
    # Confirm vehicle armed before attempting to take off
    counter=0
    while not vehicle.armed and counter<intentos:      
        print " Waiting for arming..."
        vehicle.armed = True        
        counter+=1
    return vehicle.armed

def disarmit():
    global vehicle
    print "Changing to STABILIZE..."
    setmode("STABILIZE")
    print "Attempting to disarm..."
    vehicle.armed   = False
    print "Armed: %s" % vehicle.armed
    print "Ready"
    return vehicle.armed

def setmode(newmode):
    """
    'Available modes: ', ['RTL', 'POSHOLD', 'LAND', 'OF_LOITER', 'STABILIZE', 'AUTO', 'GUIDED', 'DRIFT', 'FLIP', 'AUTOTUNE', 'ALT_HOLD', 'LOITER', 'POSITION', 'CIRCLE', 'SPORT', 'ACRO']
    """
    global vehicle
    if newmode!="AUTO":
##        while vehicle.mode!=newmode:
##            vehicle.mode = VehicleMode(newmode)
##            print vehicle.mode
##            time.sleep(5)
        vehicle.mode = VehicleMode(newmode)
    else:
        vehicle.mode = VehicleMode(newmode)
    time.sleep(3)
    if vehicle.mode==newmode:
        print "SETMODE -> Mode changed to: ",vehicle.mode," closing method setmode()"
    else:
        print "Unable to change mode."
    return vehicle.mode	
    
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

def arm_and_takeoff(aTargetAltitude):
    global vehicle
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    armit()
    print "Changing to GUIDED",vehicle.mode.name	
    setmode("GUIDED")
    if vehicle.armed==True:
        print "Taking off!"
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
	    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
	    #  after Vehicle.simple_takeoff will execute immediately).
        while vehicle.location.global_relative_frame.alt<aTargetAltitude*0.95:
            print " Altitude: ", vehicle.location.global_relative_frame.alt
            time.sleep(1)
	    #Break and return from function just below target altitude.        
            #print "Reached target altitude, changing to ALT HOLD"    
	    #setmode("LOITER")
    print "exit arm_and_takeoff"

def rcoverrideTHR():
    print "WARNING: RC OVERRIDE GAS!!!! 20%"
    vehicle.channels.overrides['3'] = 1100
    time.sleep(3)
    vehicle.channels.overrides = {}

#------------------------------------------------------------------------------------------------------------------
        
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

#------------------------------------------------------------------------------------------------------------------

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission()
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)
        
def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName)

    print "\nUpload mission from a file: %s" % import_mission_filename
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission'
    vehicle.commands.upload()

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    return cmds

def print_mission(waypoints):
    for waypoint in waypoints:
        print waypoint

def start_mission():
    #global missionON
    #if missionON==False:
    print "Calling setmode(AUTO)"
    setmode("AUTO")
    time.sleep(3)
    print "checking if mode is AUTO"
    if vehicle.mode=="AUTO":
	print "AUTO mode detected, buckle up!"	
        rcoverrideTHR()
    else:
        print "Unable to change Drone Mode to AUTO"
    #else:
    #    print "Unable to start, mission already running."
        
#------------------------------------------------------------------------------------------------------------------

#------------------------------------------------------LOOP FUNCTIONS----------------------------------------------
def funcion_conectar():
    global vehicle
    global connected
    global pause_script
    
    print "Connecting to drone..."
    engine.say("Connecting to drone...")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.say(" ")
    engine.runAndWait()
    while connected==False:
        try:
	    #vehicle=connect(args.connect, wait_ready=['system_status','gps_0', 'armed', 'mode'])
            #vehicle=connect('COM6', baud=57600, wait_ready=['system_status','gps_0', 'armed', 'mode'])
            vehicle=connect('/dev/ttyUSB0',baud=57600,wait_ready=True)
            connected=True
        except Exception:
            print "Unable to connect... Retrying"
            engine.say("Unable to connect... Retrying")
            engine.say(" ")
            engine.say(" ")
            engine.runAndWait()
            time.sleep(0.5)        
    vehicle.add_attribute_listener('armed',attribute_callback)
    vehicle.add_attribute_listener('mode',attribute_callback)
    vehicle.parameters.add_attribute_listener('gps0.fix_type',attribute_callback)
    vehicle.parameters.add_attribute_listener('system_status.state',attribute_callback)
    #vehicle.add_message_listener('HEARTBEAT',beep)

def funcion_armar():
    print "Arming Drone"
    armed=armit()
    time.sleep(1)
    if armed==True:
        print "DRONE ARMED"
    else:
        print "unable to ARM"

def funcion_mision():
    print "Starting mission..."
    start_mission()

def funcion_aterrizar():
    print "Landing Drone"
    setmode("LAND")
    print "mode changed"

#------------------------------------------------------------------------------------------------------------------        
if test==True:
    # Connect to the Vehicle
    print "Test mode, currently deleted =)"
else:
    printMenu=True
    while True:
        if connected==False:
            engine = pyttsx.init()
            #updateLeds()
            print "Connecting / Reconnecting ..."
            engine.say("Connecting / Reconnecting ...")
            engine.say(" ")
            engine.runAndWait()
            funcion_conectar()
            connected=True 
            print "Reconnected!"
	    vehiclestate()	
            #Agrego el listener para checkear latidos
            @vehicle.on_attribute('last_heartbeat')
            def listener(self, attr_name, value):
                global pause_script
                if value > 25 and not pause_script:
                    print "Pausing script due to bad link"
                    pause_script=True;
                if value < 25 and pause_script:
                    pause_script=False;
                    print "Un-pausing script"
			
            #Create a message listener for all messages.
            @vehicle.on_message('*')
            def listener(self, name, message):
                #print 'message: %s' % message
##		if name=="COMMAND_ACK":
##                    print 'COMMAND_ACK received :', message
##		if name=="SET_MODE":
##                    print 'SET_MODE received :', message
##                if name=="MAV_CMD_COMPONENT_ARM_DISARM":
##                    print 'MAV_CMD_COMPONENT_ARM_DISARM received :', message
##                if name=="SYS_STATUS":
##                    print 'PAQUETE: ',name,' => DATOS :', message
                pass
                    
            @vehicle.on_message('HEARTBEAT')
            def my_method(self, name, message):
##                print 'HEARTBEAT received....  =>', message
##                print '\t Tipo Vehiculo :', message.type
##                print '\t TYPE: ', message.autopilot
##                print '\t BASE_MODE:', message.base_mode
##                print '\t CUSTOM MODE: ', message.custom_mode
##                print '\t SYSTEM STATUS: ', message.system_status
##                print '\t MAVLINK VERSION: ', message.mavlink_version
                pass
                
        if manual==True:
            if pause_script==False:
                #MODO CONSOLA
                if printMenu==True:
                    menu = {}
                    menu['1']="Connect/Disconnect Drone"
                    menu['2']="ARM/DISARM"
                    menu['3']="TAKEOFF/LAND"
                    menu['4']="START MISSION"
                    menu['5']="Drone data"
                    menu['6']="Drone status"

                    print "\n Menu de Control \n"
                    print "armed:",armed,"connected:",connected
                    
                    options=menu.keys()
                    options.sort()
                    for entry in options:
                        print entry, menu[entry]
                #Read OPTION
                selection=0
                selection=raw_input("\n Please Select:")
                if selection =='1':
                    #Connect/Disconnect Drone
                    if connected==False:
                        connected=connectDrone()
                        if connected!=None:
                            connected=True
                            checkDrone()
                        else:
                            connected=False
                    else:
                        disconnectDrone()
                        connected=False
                    print "1 ready"
                elif selection == '2':
                    #ARM/DISARM
                    if armed==False:
                        funcion_armar()
                elif selection == '3':
                    #TAKEOFF/LAND
                    funcion_aterrizar()
                elif selection == '4':
                    #START MISSION
                    funcion_mision()
                elif selection == '5':
                    vehiclestate()
                if selection!=0:
                    printMenu=True
            else:
                #vehicle.close()
                #connected=False
                pass
        else:
            updateLeds()
            #MODO BOTONES                
            if pause_script==False:
                #ARM FUNCTION CALL
                if GPIO.input(armBUTTON)==0 and armed==False:
                    t0=datetime.datetime.now()
                    while GPIO.input(armBUTTON)==0 and armed==False:
                        t=datetime.datetime.now()
                        if (t-t0).seconds>armDelay:        
                            funcion_armar()
                #MISSION FUNCTION CALL with previous GPS lock check
                if  GPIO.input(missionBUTTON)==0 and int(vehicle.gps_0.fix_type)>=3 and armed==True:
                    funcion_mision()
                    if vehicle.mode!="AUTO":
                        print "unable to change mode"
                    else:
                        print"Mission should be already running"
                    time.sleep(2)
                #LAND FUNCTION CALL        
                if GPIO.input(landBUTTON)==0: 
                    funcion_aterrizar()
                #ACTIALIZAR ESTADO Y LEDS    
                #armed=vehicle.armed	
                #updateLeds()
                #print vehiclestate()
            else:
                #vehicle.close()
                #connected=False
                updateLeds()
        time.sleep(0.1)



