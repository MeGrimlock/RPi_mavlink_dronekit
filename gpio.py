import RPi.GPIO as GPIO
import time

#------------------------------------------Inicializacion--------------------------------
GPIO.setmode(GPIO.BOARD)

GPIO.setup(31, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)

#G#PIO.setup(33,GPIO.IN, pull_up_down=GPIO.PUD_UP) 
#GPIO.setup(35,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(37,GPIO.IN, pull_up_down=GPIO.PUD_UP)

#---------------------------------------------PROGRAMA-----------------------------------

def blink(loops):
	counter=0
	while counter<loops:
                print "loop"
		time.sleep(1)
		print "on"
		GPIO.output(31 , True)
		GPIO.output(33, True)
		GPIO.output(35, True)
		GPIO.output(37, True)
		time.sleep(3)
		print "off"
		GPIO.output(31, False)
		GPIO.output(33, False)
		GPIO.output(35, False)
		GPIO.output(37, False)
                time.sleep(3)
		counter+=1

GPIO.output(31, False)
GPIO.output(33, False)
GPIO.output(35, False)
GPIO.output(37, False)

while True:
	blink(10)
