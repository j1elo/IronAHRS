# This is a test/3D visualization program for the Pololu MinIMU-9 + Arduino
# AHRS, based on "Test for Razor 9DOF IMU" by Jose Julio, copyright 2009.

# This script needs VPython, pyserial and pywin modules

# First Install Python 2.6.4 (Python 2.7 also works)
# Install pywin from http://sourceforge.net/projects/pywin32/
# Install pyserial from http://sourceforge.net/projects/pyserial/files/
# Install VPython from http://vpython.org/contents/download_windows.html

import vis
import math
from math import sin, cos
import time
import sys, signal
import serial

def signal_handler(signal, frame):
    """Handle CTRL+C signal."""
    
    com.close()
    print("CTRL+C received. Exit now!")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Serial port identifier.
COM_PORT = "COM6"

# Global access to serial port.
com = serial.Serial()

CONST_DEG2RAD = math.pi / 180.0


# Main scene
scene = vis.display(title="IronAHRS")
scene.range=(1.2,1.2,1.2)
#scene.forward = (0,-1,-0.25)
scene.forward = (1,0,-0.25)
scene.up=(0,0,1)

# Second scene (Roll, Pitch, Yaw)
scene2 = vis.display(title="IronAHRS", x=0, y=0, width=500, height=200, center=(0,0,0), background=(0,0,0))
scene2.range=(1,1,1)
scene.width=500
scene.y=200

# Roll, Pitch, Yaw
scene2.select()
cil_roll = vis.cylinder(pos=(-0.4,0,0),axis=(0.2,0,0),radius=0.01,color=vis.color.red)
cil_roll2 = vis.cylinder(pos=(-0.4,0,0),axis=(-0.2,0,0),radius=0.01,color=vis.color.red)
cil_pitch = vis.cylinder(pos=(0.1,0,0),axis=(0.2,0,0),radius=0.01,color=vis.color.green)
cil_pitch2 = vis.cylinder(pos=(0.1,0,0),axis=(-0.2,0,0),radius=0.01,color=vis.color.green)
#cil_course = vis.cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=vis.color.blue)
#cil_course2 = vis.cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=vis.color.blue)
arrow_course = vis.arrow(pos=(0.6,0,0),color=vis.color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)

#Roll,Pitch,Yaw labels
vis.label(pos=(-0.4,0.3,0),text="Roll",box=0,opacity=0)
vis.label(pos=(0.1,0.3,0),text="Pitch",box=0,opacity=0)
vis.label(pos=(0.55,0.3,0),text="Yaw",box=0,opacity=0)
vis.label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=vis.color.yellow)
vis.label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=vis.color.yellow)
vis.label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=vis.color.yellow)
vis.label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=vis.color.yellow)
vis.label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=vis.color.yellow)
vis.label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=vis.color.yellow)
vis.label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=vis.color.yellow)
vis.label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=vis.color.yellow)

L1 = vis.label(pos=(-0.4,0.22,0),text="-",box=0,opacity=0)
L2 = vis.label(pos=(0.1,0.22,0),text="-",box=0,opacity=0)
L3 = vis.label(pos=(0.7,0.3,0),text="-",box=0,opacity=0)

# Main scene objects
scene.select()
# Reference axis (x,y,z)
vis.arrow(color=vis.color.green,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
vis.arrow(color=vis.color.green,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
vis.arrow(color=vis.color.green,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)
# labels
vis.label(pos=(0,0,0.8),text="Pololu MinIMU-9 + Arduino AHRS",box=0,opacity=0)
vis.label(pos=(1,0,0),text="X",box=0,opacity=0)
vis.label(pos=(0,-1,0),text="Y",box=0,opacity=0)
vis.label(pos=(0,0,-1),text="Z",box=0,opacity=0)
# IMU object
platform = vis.box(length=1, height=0.05, width=1, color=vis.color.blue)
p_line = vis.box(length=1,height=0.08,width=0.1,color=vis.color.yellow)
plat_arrow = vis.arrow(color=vis.color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)


# Data source: serial port connection.
print("Opening serial port...")
# Configure and open the Serial port.
com.baudrate = 115200
com.bytesize = serial.EIGHTBITS
com.parity = serial.PARITY_NONE
com.stopbits = serial.STOPBITS_ONE
com.timeout = 1
com.xonxoff = True
com.port = COM_PORT
try:
	com.open()
except serial.SerialException:
	print(
		"Oops! The device '{}' cannot be found "
		"or configured.".format(com.name))
	sys.exit()
print("Opening serial port... OK")

print("Setup Sensor Board...")
# Configure Board output parameters.
com.write(b"qot")  # Setup "YawPitchRoll-Text" output mode.
com.flushInput()  # Clear input buffer up to here.
#time.sleep(0.1)  # Give it time to change output mode. No parece que funcione.
print("Setup Sensor Board... OK")


roll=0
pitch=0
yaw=0
finish = False
while not finish:
	line = com.readline()
	#print("Input: {}".format(line.strip(b"\r\n")))
	line = line.strip(b"\r\n").decode("utf-8")
	#if line.find("#YPR=") != -1:  # Filter out incomplete (invalid) lines.
	if line:  #  Run the loop while 'line' is not an empty string.
		line = line.replace("#YPR=", "")  # Delete line header.
		print(line)
		words = line.split(",")
		if len(words) > 2:
			try:
				yaw = float(words[0]) * CONST_DEG2RAD
				pitch = float(words[1]) * CONST_DEG2RAD
				roll = float(words[2]) * CONST_DEG2RAD
			except:
				print("Invalid line")

			axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
			up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
			platform.axis=axis
			platform.up=up
			platform.length=1.0
			platform.width=0.65
			plat_arrow.axis=axis
			plat_arrow.up=up
			plat_arrow.length=0.8
			p_line.axis=axis
			p_line.up=up
			cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
			cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
			cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
			cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
			arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)
			L1.text = str(float(words[0]))
			L2.text = str(float(words[1]))
			L3.text = str(float(words[2]))        
com.close()
