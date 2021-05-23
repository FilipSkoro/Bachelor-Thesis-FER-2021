#!/usr/bin/env python3

from Task import Task
from Bid import Bid
import rospy
import copy
from pia_ros.srv import ProvideTauc, ProvideTaucResponse, DeclareWinner, DeclareEnd

class Auctioneer():
	# Object created with a type of this class represents auctioneer and its attributes.
	# This class provides some useful methods and functions used for Simple Iterated Auction algorithm.

	def __init__(self):
		# This function initializes object of the class Auctioneer
		# and creates list called tasks in which all of the tasks needed
		# for auction will be stored once they are loaded from server.

		print("\n=========================================")
		print("\n*** Simple Iterated Auction algorithm ***")
		print("\n=========================================\n")
		self.tasks = []

	def task_to_object(self):
		# This function creates and returns object of a type Task by using paramteres
		# available on server under name /tasks. Server paramateres are names and attributes 
		# uploaded by pia_ros.launch file from Tasks.yaml file.

		tasks = rospy.get_param("/tasks")

		for i in range(len(tasks)):
			location = tasks[i]["location"].split(", ")
			self.tasks.append(Task(tasks[i]["name"],tasks[i]["resources"],tasks[i]["duration"],tasks[i]["precedence"],location[0],location[1]))

	def sIA_function(self):
		# This function represents The Simple Itareted Auction algorithm. It provides
		# list of tasks that need to be assigned to the agents. After it recieves bids from
		# the agents it calls choose_bid function that decides which bids are accepted and which
		# are not.

		self.TF = []
		self.Tauc = []
		self.bids = []
		TaucMsg = []

		self.TF = self.tasks[:]
		self.Tauc = self.TF[:]

		print(f'Tasks on auction: {self.Tauc}\n')

		while self.Tauc:
			TaucMsg.clear()
			for task in self.Tauc:
				TaucMsg.append(task.to_msg())

			for name in ['robot1', 'robot2']:																# <=== ovo ce trebati pametnije napraviti - M
				rospy.wait_for_service(f'/{name}/provide_tauc')
				try:
					self.resp1 = ProvideTaucResponse()
					provide_tauc = rospy.ServiceProxy(f'/{name}/provide_tauc', ProvideTauc)
					self.resp1 = provide_tauc(TaucMsg)
					self.bids.append(Bid(Task.to_task(self.resp1.bid),self.resp1.time,self.resp1.robot_name))
				except rospy.ServiceException as exc:
					print("Service did not process request: " + str(exc))

				print("Bidder name: {0}".format(self.resp1.robot_name),end='\n')
				print("Bid: {0}".format(self.resp1.bid.name),end='\n')
				print("Time: {0} seconds\n".format(self.resp1.time))

			self.choose_bid()

			if self.Tauc:
				print(f"=========================================\n")
				print(f"Tasks still left on auction: {self.Tauc}\n")
			else:
				print("=========================================\n")
				for name in ['robot1', 'robot2']:															# <=== ovo ce trebati pametnije napraviti - M
					rospy.wait_for_service(f'/{name}/declare_end')
					try:
						declare_end = rospy.ServiceProxy(f'/{name}/declare_end', DeclareEnd)
						declare_end(True)
					except rospy.ServiceException as exc:
						print(end='')

	def choose_bid(self):
		# This function declares which bids are accepted and which are not.
		# If there is only one bid for a certain task then it is surely going to be
		# accepted. If there are two or more bids for the same task, the bid with the lowest
		# schedule time will be accepted while other bids will be declined. Tasks are being removed
		# from the auction list after they have been assigned.

		while self.bids:
			robot_name = copy.deepcopy(self.bids[0].robot_name)
			task_name = copy.deepcopy(self.bids[0].bid.name)
			task_time = copy.deepcopy(self.bids[0].time)

			self.bids.remove(self.bids[0])

			if self.bids:
				for i in range(len(self.bids)):
					if self.bids[i].bid.name == task_name:
						if self.bids[i].time < task_time:
							rospy.wait_for_service(f'/{robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							task_time = copy.deepcopy(self.bids[i].time)
							robot_name = copy.deepcopy(self.bids[i].robot_name)
							self.bids.remove(self.bids[i])
						else:
							rospy.wait_for_service(f'/{self.bids[i].robot_name.lower()}/declare_winner')
							try:
								declare_winner = rospy.ServiceProxy(f'/{self.bids[i].robot_name.lower()}/declare_winner', DeclareWinner)
								declare_winner(False)
							except rospy.ServiceException as exc:
								print(end='')

							self.bids.remove(self.bids[i])
					else:
						continue

			rospy.wait_for_service(f'/{robot_name.lower()}/declare_winner')
			try:
				declare_winner = rospy.ServiceProxy(f'/{robot_name.lower()}/declare_winner', DeclareWinner)
				declare_winner(True)
			except rospy.ServiceException as exc:
				print(end='')

			self.remove_task(task_name)

	def remove_task(self, task_name):
		# This function removes task from the list of tasks that are being
		# auctioned after it has been assigned.

		for task in self.Tauc:
			if task.name == task_name:
				self.Tauc.remove(task)
			else:
				continue

if __name__ == "__main__":
	# This is 'main function' used for initializing node and creating
	# object of a type Auctioneer. It calls function for creating Task
	# objects and Simple Iterated Auction function.

	rospy.init_node('Auctioneer')

	try:
		A = Auctioneer()
		A.task_to_object()

		A.sIA_function()

	except rospy.ROSInterruptException: pass

##	==========================================
##	*** Simple Iterated Auction algorithm ***
##	==========================================
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