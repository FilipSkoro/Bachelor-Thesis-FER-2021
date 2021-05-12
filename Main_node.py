#!/usr/bin/env python3

import rospy
from Auctioneer_node import Auctioneer
from Bidder_node import Robot

def agent_to_object(file):
		lines = file.readlines()
		attr = []
		R = []
		lines = lines[:lines.index("tasks:\n")]
		for line in lines:
			if line == "\n" or line == "agents:\n":
				continue
			else:
				pom = line.split(": ")
				if pom[0] == '    location':
					pom2 = pom[1].split(", ")
					attr.append(pom2[0])
					attr.append(pom2[1])
					R.append(Robot(attr[0], attr[1], attr[2], attr[3]))
					attr.clear()
				else:
					attr.append(pom[1])
		return R

if __name__ == "__main__":
	rospy.init_node('provide_tauc')

	try: 
		file = open('Tasks.yaml', 'r')
		A = Auctioneer()
		R = []
		R = agent_to_object(file)
		A.task_to_object(file)

		A.sIA_function(R)

		file.close()

	except rospy.ROSInterruptException: pass