#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from baxter_core_msgs.msg import HeadPanCommand, HeadState
import sys, os, time
import baxter_dataflow 


class BaxterScreen():
	
	def __init__(self, logStatus=True):
		if logStatus:
			self.log("Initializing node for Baxter's head screen")
		try:
			rospy.init_node("baxterScreen")
		except rospy.ROSException, e:
			if 'rospy.init_node() has already been called with different arguments' in str(e):
				pass
			else:
				raise
		self._screenPub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1, latch=True)
		self._headPub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=5, latch=True)
		self._headAngle = 1000
		self._nodPub = rospy.Publisher('/robot/head/command_head_nod', Bool, queue_size=2, latch=True) 
		self._nodding = False
		
	def __enter__(self):
		return self
		
	def __exit__(self, exc_type, exc_value, traceback):
		pass
		
	def _on_head_state(self, msg):
		self._headAngle = msg.pan
		self._nodding = msg.isNodding
		
	def showImage(self, imageFile=None, imageMsg=None, logStatus=True):
		if imageFile is not None:
			if logStatus:
				self.log("Showing image file %s" % imageFile)
			img = cv2.imread(imageFile)
			if img is None:
				if logStatus:
					self.log("INVALID IMAGE file %s" % imageFile)
			else:
				imageMsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
			self._screenPub.publish(imageMsg)
			rospy.sleep(0.5)
		elif imageMsg is not None:
			if logStatus:
				self.log("Showing image message")
			self._screenPub.publish(imageMsg)
			rospy.sleep(0.5)
			
	def showVideo(self, videoFile, fps_vid=25, fps_display=10, logStatus=True):
		fpsRate = rospy.Rate(fps_display)
		videoCapture = cv2.VideoCapture(videoFile)
		if videoCapture is None:
			return
		if logStatus:
			self.log("Playing video file %s" % videoFile)
		while True:
			try:
				frame = None
				for i in range(fps_vid/fps_display):
					ret, frame = videoCapture.read()
				imageMsg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
				self._screenPub.publish(imageMsg)
				fpsRate.sleep()
			except:
				break
		videoCapture.release()
		cv2.destroyAllWindows()
		if logStatus:
			self.log("Finished playing video file %s" % videoFile)
		
	def moveHead(self, angle, speed=50, degrees=True, tolerance=3*3.14159/180.0, logStatus=True):
		if logStatus:
			self.log("Moving head pan to %s %s" % (angle, "degrees" if degrees else "radians"))
		if degrees:
			angle = angle*3.14159/180.0
		headSub = rospy.Subscriber('/robot/head/head_state', HeadState, self._on_head_state)
		headCmnd = HeadPanCommand(angle, speed, HeadPanCommand.REQUEST_PAN_VOID) # REQUEST_PAN_VOID means don't change pan request setting
		self._headPub.publish(headCmnd)
		count = 0
		while(abs(self._headAngle - angle) > tolerance):
			time.sleep(0.01)
			count = count + 1
			if count > 2/0.01:
				if logStatus:
					self.log("ERROR MOVING HEAD did not reach desired tolerance in 2 seconds")
				break
				
	def nod(self, times=3):
		headSub = rospy.Subscriber('/robot/head/head_state', HeadState, self._on_head_state)
		for i in range(times):
			self._nodPub.publish(True)
			# wait for nod to start
			while not self._nodding:
				time.sleep(0.05)
			# wait for nod to complete
			while self._nodding:
				time.sleep(0.05)
		
	def log(self, msg):
		rospy.loginfo(rospy.get_caller_id() + ": " + msg)
		
if __name__=='__main__':
	screen = BaxterScreen()
	count = 0
	for arg in sys.argv:
		try:
			(keyword, value) = arg.split('=')
			if len(value) == 0:
				continue
			if 'image' in keyword:
				image = value.strip()
				screen.showImage(image)
				count = count+1
			elif 'video' in keyword:
				video = value.strip()
				screen.showVideo(video)
				count = count+1
			elif 'degrees' in keyword:
				angle = float(value.strip())
				screen.moveHead(angle)
				count = count+1
			elif 'radians' in keyword:
				angle = float(value.strip())
				screen.moveHead(angle, degrees=False)
				count = count+1
			elif 'nod' in keyword:
				times = int(value.strip())
				screen.nod(times)
				count = count+1
		except ValueError:
			continue
	if count == 0:
		print("Usage: BaxterScreen.py image=path/to/image video=path/to/video degrees=angle radians=angle nod=3")
		print("  Any subset of arguments may be provided, and all will be treated separately in order provided")
		print("  image and video will show the image or video on the screen")
		print("  degrees and radians will pan the head to that angle")
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
		
