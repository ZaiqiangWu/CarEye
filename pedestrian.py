import time
import cv2

class pedestrian():
	def __init__(self,location,size,confidence):
		self.confidence=confidence
		self.location=location #location is a tuple of the form (x,y)
		self.size=size #size of the bounding box
		self.prev_location=[]
		self.gaze=0 #when this is 1, pedestrian is being gazed at!
		self.time=time.time()		
		self.gazetime=0.0
		self.superior=False
		self.eyes=(location[0], int(location[1] - size[1] * 0.3))
		self.id=0
	def update_eyes(self,location):
		if (not isinstance(location, float)):
			self.eyes=location
	def prioritize(self,bool):
		self.superior=bool
	def addstarttime(self):
		self.starttime=time.time()

	def add_location(self,location):
		if len(self.prev_location)==5:
			for i in range(len(self.prev_location)-1):
				self.prev_location[i]=self.prev_location[i+1]
			self.prev_location[-1]=location
		else:     
			self.prev_location.append(location)
	
	def updategazetime(self):
		if (self.gaze==1):
			#always call updatetime after updategazetime
			self.gazetime=time.time()-self.starttime

	def updatetime(self):
		self.time=time.time()
	
	def changegazestatus(self,status):
		self.gaze=status
		if status==0:
			self.gazetime=0

class pedes_container:
	def __init__(self):
		self.plist=[]

	def update(self, results):
		new_plist = []
		for i in range(len(results)):
			location = (results[i][2][0], results[i][2][1])
			size = (results[i][1][2] - results[i][1][0], results[i][1][3] - results[i][1][1])
			confidence = results[i][0]
			new_pedestrian = pedestrian(location, size, confidence)
			new_plist.append(new_pedestrian)
		if len(self.plist)<len(new_plist):
			for i in range(len(self.plist)):
				pass