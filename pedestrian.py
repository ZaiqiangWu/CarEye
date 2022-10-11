import time
import cv2
import functools


class pedestrian():
	def __init__(self, location, size, confidence):
		self.confidence = confidence
		self.location = location  # location is a tuple of the form (x,y)
		self.size = size  # size of the bounding box
		self.prev_location = []
		self.eyes = (location[0], int(location[1] - size[1] * 0.3))

	def update_eyes(self, location):
		if (not isinstance(location, float)):
			self.eyes = location

	def add_location(self, location):
		if len(self.prev_location) == 5:
			for i in range(len(self.prev_location) - 1):
				self.prev_location[i] = self.prev_location[i + 1]
			self.prev_location[-1] = location
		else:
			self.prev_location.append(location)


def cmp1(a, b):
	res = 0
	s1 = a.size[0] * a.size[1]
	s2 = b.size[0] * b.size[1]
	if s1 > s2:
		res -= 1
	elif s1<s2:
		res += 1
	else:
		res=0
	return res


class pedes_container:
	def __init__(self):
		self.plist = []
		self.time_out = 5.0
		self.last_time = time.time()
		self.current_time = 0.0
		self.focus_id = 0

	def update(self, results):
		new_plist = []
		for i in range(len(results)):
			location = (results[i][2][0], results[i][2][1])
			size = (results[i][1][2] - results[i][1][0], results[i][1][3] - results[i][1][1])
			confidence = results[i][0]
			new_pedestrian = pedestrian(location, size, confidence)
			new_plist.append(new_pedestrian)
		new_plist.sort(key=functools.cmp_to_key(cmp1))
		self.plist = new_plist

	def get_eye_direction(self):
		self.current_time = time.time()
		if self.current_time - self.last_time > self.time_out:
			self.focus_id = (self.focus_id + 1) % len(self.plist)
			self.last_time = self.current_time
		if self.focus_id>=len(self.plist):
			self.focus_id=0
		return self.plist[self.focus_id].eyes

	def any(self):
		return len(self.plist) > 0
