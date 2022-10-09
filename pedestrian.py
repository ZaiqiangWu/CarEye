import time

class pedestrian():
	def __init__(self,location,size,confidence,eyes):
		self.confidence=confidence
		self.location=location #location is a tuple of the form (x,y)
		self.size=size #size of the bounding box
		self.prev_location=[]
		self.gaze=0 #when this is 1, pedestrian is being gazed at!
		self.time=time.time()		
		self.gazetime=0.0
		self.superior=False
		self.eyes=eyes
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