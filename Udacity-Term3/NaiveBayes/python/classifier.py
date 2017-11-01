# *_ coding: UTF-8 _*
# 1. 给每个label计算每个feature的高斯函数（每个feature有 N个高斯函数）
# 2. 给定一个train，计算各feature的N个高斯函数对应的概率
# 	 把各feature代表的相同label相乘，得到最大概率的label 
from pandas.core.frame import DataFrame
import numpy as np

class GNB(object):

	def __init__(self):
		self.possible_labels = ['left', 'keep', 'right']

		self.gaussians = {}
	
	def gaussians_calc(self, x,mu,sigma):
		return np.exp(-((x - mu)**2)/(2*sigma**2)) / (sigma * np.sqrt(2*np.pi))

	def train(self, data, labels):
		"""
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
		"""
		data = DataFrame(data)
		data['labels'] = labels
		#print data.head()
		for label in self.possible_labels:
			df = data.loc[(data['labels'] == label)]
			mean_list = np.array(df.mean())
			sum_list = np.array(df.sum())
			std_list = np.array(df.std())
			self.gaussians[label] = [zip(mean_list, std_list)]
			print mean_list
	def predict(self, observation):
		"""
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
		p_max = 0
		predict = None
		for label in self.possible_labels:
			possiblity = 1
			label_gaussian = self.gaussians.get(label)
			for i in range(len(observation)):
				(mean, std) = label_gaussian[0][i]
				value = observation[i]
				possiblity *= self.gaussians_calc(value, mean, std)
			if p_max < possiblity:
				p_max = possiblity
				predict = label

		return predict

