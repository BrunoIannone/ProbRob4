
import numpy as np
import matplotlib.pyplot as plt
import json
import time
class DatasetHandler():
	"""Utility class for handling the dataset
	"""
	def __init__(self,path,header_length=10):
		"""DatasetHandler init class

		Args:
			path (str): dataset.txt path
			header_length (int, optional): Number of header lines to skip. Defaults to 10.
		"""
		self.path = path
		self.header_length = header_length
    
	def view_traj(self):
		"""Plot trajectory
		"""
		f = open(self.path)

		lines = f.read().splitlines()

		c = 0
		x = []
		y = []
		for l in lines:
			c += 1
			if(c < self.header_length):
				continue
			tokens = l.split(":")
			tracker_pose = tokens[-1].strip()
			xy = tracker_pose.split(" ")
			x.append(float(xy[0]))
			y.append(float(xy[1]))

		# x_np = np.asarray(x)
		# y_np = np.asarray(y)
		fig = plt.figure()
		ax = fig.add_subplot(111)
		ax.scatter(x, y)
		ax.axis("equal")
		plt.show()

	def convertToOctave(self,dest):
		"""Convert dataset.txt to a matrix for octave usage

		Args:
			dest (str): .txt savepath
		"""
		f = open(self.path)
		f_w = open(dest+"dataset_octave.txt","w")

		lines = f.read().splitlines()

		c = 0
		
		for l in lines:
			c += 1
			if(c < self.header_length):
				continue
			
			splitted_line = l.split(":")
			full_splitted_line = []
			for elem in splitted_line:
				elem = elem.strip().split(" ")
				full_splitted_line.extend(elem)
			#print(full_splitted_line)
			#time.sleep(2)
			str_ = ""
			for elem in full_splitted_line:
				#print(elem)
				#time.sleep(2)
					
				try:
					elem_ = float(elem)
					str_+=elem + " "
				except ValueError:
					continue

			str_+="\n"	
			f_w.write(str_)

		
