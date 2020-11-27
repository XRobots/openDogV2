import RPi.GPIO as GPIO
import jetson.inference
import jetson.utils

import time

import argparse
import sys

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

#setup GPIO pins

GPIO.setmode(GPIO.BCM) #RaspPi pin numbering

GPIO.setup(18, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(18, GPIO.HIGH)
GPIO.setup(17, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(17, GPIO.HIGH)
GPIO.setup(16, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(16, GPIO.HIGH)
GPIO.setup(20, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(20, GPIO.HIGH)
GPIO.setup(21, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(21, GPIO.HIGH)

def back():
	GPIO.output(18, GPIO.LOW)
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.HIGH)
	print("back")

def forward():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(17, GPIO.LOW)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.HIGH)
	print("forward")

def left():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(16, GPIO.LOW)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.HIGH)
	print("left")

def right():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.LOW)
	GPIO.output(21, GPIO.HIGH)
	print("right")

def up():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.LOW)
	print("up")

def nothing():
	GPIO.output(18, GPIO.HIGH)
	GPIO.output(17, GPIO.HIGH)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.HIGH)
	print("nothing")	



# declare variables as global and that

global index
global width
global location
global confidence
index = 0
width = 0
location = 0
condifence = 0;

# process frames until the user exits
while True:	

	# capture the next image
	img = input.Capture()

	# detect objects in the image (with overlay)
	detections = net.Detect(img, overlay=opt.overlay)

	# print the detections
	#print("detected {:d} objects in image".format(len(detections)))

	# check for detections, otherwise nothing	

	if(len(detections) > 0):
		print("object detected")
		for detection in detections:
			index = detections[0].ClassID
			confidence = (detections[0].Confidence)
			width = (detections[0].Width)
			location = (detections[0].Center[0])

			# print index of item, width and horizonal location

			print(index)
			print(width)
			print(location)
			print(confidence)

			# look for detections

			if (index == 1 and confidence > 0.9):
				back()

			elif (index == 2 and confidence > 0.7):
				forward()

			elif (index == 3 and confidence > 0.7):
				left()

			elif (index == 4 and confidence > 0.7):
				right()

			elif (index == 5 and confidence > 0.7):
				up()

	else:
		nothing()	# nothing is detected


	# render the image
	output.Render(img)

	# update the title bar
	output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance info
	#net.PrintProfilerTimes()

	# exit on input/output EOS
	if not input.IsStreaming() or not output.IsStreaming():
		break


