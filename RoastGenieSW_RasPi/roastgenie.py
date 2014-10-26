# PiRoaster
# This script interfaces with the aruduio roaster control version 17 & 18
# Written by Evan Graham  www.roasthacker.com
# Hardware setup:
#	pin1	3v3 power	com interface LV
#	pin6	ground		com interface GND
#	pin8	GPIO14 Tx	com interface Tx-Arduino Rx
#	pin9	ground		n/o switch for shutdown
#	pin10	GPIO15 Rx	com interface Rx-Arduino Tx
#	pin11	GPIO17		n/o switch for shutdown

# Com interface is sparkfun BOB12009 bi-directional logic level converter

# Manual shutdown behavior is defined in a separate script

import serial
import time
import datetime
import os
from os import system
import picamera
from time import strftime, gmtime, localtime

program = 'piRoaster'
version = '17.6'

# 17.4  
#	remove tmz libraries
#	allow for different plot templates when not using the camera
#	moved options section to the top


# 17.5
#	remove plot file generation (now done externally)


#17.6
#	addding/revising code to read beanmass temps


# OPTIONS
debug 		= 0					# 1 = verbose output 0 = no debug output
enable_camera 	= 0					# 1 = camera present 0 = no camera present

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
ser.open()

try:
	btSer = serial.Serial('/dev/rfcomm0', 9600, timeout=1)
	btSer.open()  #may not be required
	btEnabled=1
except:
	btEnabled=0

root_dir 	= '/mnt/cifs_share/coffeedata/'  	#For operation on the home network
#root_dir 	= '/home/etg/share/'			#Main directory (shared) for use on other networks
inv_file	= root_dir + 'inventory'		#Full path to coffee inventory file
data_file	= ''					#Full path to current roasting data file

if(enable_camera):
	plt_tmplt	= root_dir + 'plotfile.plt'	#Full path to plot file template
else:
	plt_tmplt	= root_dir + 'plotfile_nc.plt'	#This is the no camera version of the plot file

plt_file	= ''					#Full path to plot file for current roast
photo_file	= ''					#Full path to photo of roasted beans
graph_file	= ''					#Full path to graph image for current roast
base_file_name  = ''					#Base name (not including path) of files



# Global Variable Declaration	
origin_array		= []				# place to store country of orgin for coffees in inventory
farm_array		= []				# place to store originating farm for coffees in inventory
short_name_array	= []				# place to store abbreviated name for coffees in inventory
purchase_date_array	= []				# place to store date of purchase for coffees in inventory

name_index		= 0				# array index of currently selected coffee	
tFirstCrack		= 0				# time of first crack	
tStartCool		= 0				# time cooling cycle started
green_weight		= 0				# green weight of batch (grams)
roasted_weight		= 0				# roasted weight of batch (grams)
dummy			= '000'

def printDebug( msg ):
	if (debug):
		print msg
	return

def sendMsg( msg):
	"This formats and sends a serial message"
	msg = "{" + msg + "}"
	ser.write(msg)
	return

def captureImage():
	"takes photo with picamera"
	with picamera.PiCamera() as camera:
        	camera.start_preview()
        	time.sleep(2)
        	camera.capture (photo_file)
        	camera.stop_preview()


def updateInventory():
	"reads coffee inventory file"
	printDebug("received request for inventory")
	with open(inv_file, 'rt') as f:					#read the inventory file`
                del origin_array[:]                                     #clear the arrays
                del farm_array[:]
		del short_name_array[:]                                 
		del purchase_date_array[:]
                for inventory_line in f:
                	field = inventory_line.split(",")
			origin_array.append(field[0].rstrip())
			farm_array.append(field[1].rstrip())
			short_name_array.append(field[2].rstrip())			
			purchase_date_array.append(field[3].rstrip())
			printDebug(inventory_line.rstrip())
	return

try:

	updateInventory()			
	while 1:
		
	
		line = ser.readline()
		if(line): # Handles all of the communication with the Arduino
			if line.startswith('?C'): #request is to verify connection
				printDebug("received connect check")
				sendMsg("c")
			
			elif line.startswith("?D"): #roast data line
				if btEnabled:
					btSer.write('?')
					printDebug("bt sending")
					bline=""
					while (bline==""):
						bline = btSer.readline()
						printDebug(bline)
					btNow,btMin,btMax = bline.rstrip().split(",")
					print(btNow,btMin,btMax)
				else:
					printDebug(".")
				with open(data_file, 'a') as f:
					f.write(line[3:].rstrip() + ',' + btNow + ',' + btMin + ',' +  btMax + '\n') #exclude 1st 3 char
				ser.write("*d,data\n")
				field =line[3:].split(",")
				xmax=field[0]
				dummy = xmax.lstrip("0")
		
			elif line.startswith("?F"): #request for coffee farm name
				field = line.split(",")
				coffeeIndex=int(field[1])
				printDebug("Farm Request for coffee # %d" % coffeeIndex)
				msg = "f,%s" %farm_array[coffeeIndex]
				sendMsg(msg)
				printDebug(msg)
			
			elif line.startswith("?I"): #request for inventory
				field = line.split(",")
				coffeeIndex=int(field[1])
				printDebug("Short Name Request for coffee # %d" % coffeeIndex)
				msg = "i,%s" % short_name_array[coffeeIndex]
				sendMsg(msg)
				printDebug(msg)

			elif line.startswith('?N'): #request for # of coffees in inventory
				printDebug("Request received for number of coffees in inventory")
				invCnt = str(len(origin_array))
				msg="n,%s" % invCnt
				sendMsg("n,%s" % invCnt)
				printDebug(msg)

			elif line.startswith("?O"): #request for coffee origin
				field = line.split(",")
				coffeeIndex=int(field[1])
				printDebug("Origin Request for coffee # %d"  % coffeeIndex)
				msg = "o,%s" % origin_array[coffeeIndex]
				sendMsg(msg)
				printDebug(msg)
	
			elif line.startswith("?T"):
				printDebug("Date/Time Request")
				msg ="t,%s" % strftime("%Y-%m-%d %H:%M:%S", localtime()) 
				sendMsg(msg)
				printDebug(msg)
			
			elif line.startswith("?R"):
				printDebug("Starting Roast")
				yyyy = datetime.date.today().strftime("%Y")
				mm = datetime.date.today().strftime("%m")
				dd = datetime.date.today().strftime("%d")
				roast_file_name = yyyy + mm + dd
				thing=line.split(",")
				name_index=int(thing[1])
				green_weight=thing[2].rstrip()
				coffeeDir=os.listdir(root_dir)			#get dir listing
				fileArray=[]
				i=0
				roast_file_index=0
				for element in coffeeDir:			#now find last file
					try:
						if(int(element[0:8])==int(roast_file_name)):
							if(int(element[8:]) > roast_file_index):
								roast_file_index = int(element[8:])
					except ValueError:
						next 

				roast_file_index=roast_file_index+1		#new file name is 1 more than last file on this date
				base_file_name = roast_file_name + str(roast_file_index)
				data_file = root_dir + roast_file_name + str(roast_file_index)
				plt_file = data_file + '.plt'
				graph_file = data_file + '.png'
				photo_file = data_file + '.jpg'
				with open(data_file, 'w') as f:			#write roasting data to the file
					f.write(strftime("%Y-%m-%d %H:%M:%S", localtime()) + '\n')
					f.write(program + " " + version + '\n')
					f.write('g_weight,'+ str(green_weight) + '\n')
					f.write(origin_array[name_index] + '\n')
					f.write(farm_array[name_index] + '\n')
					f.write('purchased,' + purchase_date_array[name_index] + '\n')
				ser.write("*r,responded\n")

			elif line.startswith('?U'): #request to re-read inventory data file
				printDebug("Reading inventory file")
				updateInventory()
				ser.write("*u:OK\n")
			
			elif line.startswith("?X"):
				footer=line.split(",")
				with open(data_file, 'a') as f:
					f.write('roast,' + footer[1].rstrip() + '\n')
					f.write('crack,' + footer[2].rstrip() + '\n')
					f.write('cool,' + footer[3].rstrip() + '\n')
					f.write('r_weight,' + footer[4].rstrip() + '\n')
					tFirstCrack = int(footer[1].rstrip())
					tStartCool = int(footer[2].rstrip()) + int(footer[1].rstrip())
					tDurCrack = int(footer[2].rstrip())
					roasted_weight = int(footer[4].rstrip())
				printDebug("Roast is Done")
				if(enable_camera):
					captureImage()
				ser.write("*x:OK\n")

			elif line.startswith("?Q"): #request to shut down the RasPi
				 ser.write("*q:OK\n")
				 os.system("sudo shutdown -h now")
			else:
				printDebug("Unrecognized Command")

except KeyboardInterrupt:
	ser.close()
	if(btEnabled):	
		btSer.close()
