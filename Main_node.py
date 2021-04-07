#!/usr/bin/env python3

import rospy
from Auctioneer_node import Auctioneer

if __name__ == "__main__":
	
	try: 
		file = open('Tasks.yaml', 'r')
		A = Auctioneer()
		R = []
		R = A.agent_to_object(file)
		A.task_to_object(file)

		A.sIA_function(R)

		file.close()

	except rospy.ROSInterruptException: pass
