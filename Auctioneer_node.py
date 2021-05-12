#!/usr/bin/env python3

from Task import Task
import rospy
from pia_ros.srv import ProvideTauc, ProvideTaucResponse

class Auctioneer():

	def __init__(self):
		print("Aukcionar se stvorio!\n")
		self.tasks = []
		self.n = 0
		self.resp1 = ProvideTaucResponse()

	def task_to_object(self, file):
		file.seek(0)
		lines = file.readlines()
		attr = []
		lines = lines[lines.index("tasks:\n"):]
		for l in lines:
			if l == "\n" or l == "tasks:\n":
				continue
			else:
				pom = l.split(": ")
				if pom[0] == '    location':
					pom2 = pom[1].split(", ")
					attr.append(pom2[0])
					attr.append(pom2[1])
					self.tasks.append(Task(attr[0], attr[1], attr[2], attr[3], attr[4]))
					attr.clear()
					self.n = self.n + 1
				else:
					attr.append(pom[1])

	def sIA_function(self, R):
		self.TS = []
		self.TF = []
		self.F = []
		self.PC = []
		self.Tauc = []

		self.TF = self.tasks[:]
		#while len(self.TS) < self.n:
		self.Tauc = self.TF[:]

		rospy.wait_for_service('provide_tauc')
		provide_tauc = rospy.ServiceProxy('provide_tauc', ProvideTauc)
		try:
			self.resp1 = provide_tauc(self.Tauc)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

		print("{0}".format(self.resp1.robot_name),end='')
		print("Bid: {0}".format(self.resp1.bid.name),end='')
		print("Time: {0} seconds.\n".format(self.resp1.time))

		#return F

## Input: Precedence graph GP = (T,E), robots R
##	TS = ∅, TF = free(T)
##	TL = free(T \ TF), TH = T \ (TF ∪ TL)
##	Initialize arrays F, PC to zero
##	while |TS| < n do
##		c = max t ∈ TL (prio(t))
##		Tauct = {t ∈ TF : prio(t) ≥ c}
##		ModifiedSSI(Tauct, PC)
##		ReceiveFinishTimes(R,F)
##		for all t ∈ Tauct do
##			UpdatePrecGraph(t, PC, GP)
##		end for
##	end while
##	return F