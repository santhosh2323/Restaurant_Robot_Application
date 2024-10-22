#!/usr/bin/env python3

###########################################################################################################
'''
The below script is for managing orders from the kitchen and tables in cafe by a robot services based on the inputs

--------------------------------------------------------------------------------------
Topics used:
--------------------------------------------------------------------------------------
/order_list - To check the order list contains table numbers and proceed with the orders-- #rostopic pub /order_list resro_pkg/StringList "data: ['table_1', 'table_2','table_3']" 
/kitchen_confirmation - Confirmation from the kitchen to proceed further
/table_confirmation - confirmation from the table to proceed further
--------------------------------------------------------------------------------------
Services Used:
--------------------------------------------------------------------------------------
/cancel/order - To cancel the order by custom service - MSGTrigger

---------------------------------------------------------------------------------------
custom msg Used 
---------------------------------------------------------------------------------------
/StringList - Contains the list of orders -- data: ['table_1', 'table_2','table_3']
'''
############################################################################################################

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from resro_pkg.msg import StringList
from resro_pkg.srv import MSGTrigger, MSGTriggerResponse



locations = {
    "home": {"position": [-2.2577691113909597, 0.002580362799700385], "orientation": [-4.5695188313583127e-05, 1.5640843824548405e-06, 0.011367663740167388, 0.99993]},
    "kitchen": {"position":[-0.5124504731835834, -1.796312134434916], "orientation": [-0.00014934232683484855, -0.0001283083522063081, 0.6997373484454283, 0.7144001710696866]},
    "table_1": {"position": [-0.6230987268936306, 1.0730093452885294], "orientation": [0.0001550309826845, 0.00018333132543755967, 0.7100618512845843, 0.7041392686857681]},
    "table_2": {"position": [0.5380987274439714, 1.2119816156872036], "orientation": [0.00012435766448300593, -0.00011150624566394249, -0.7073472516007641, 0.7068662092322521]},
    "table_3": {"position": [1.650627354687455, -0.623220469450734], "orientation": [-0.0003066380998337493, 4.23440675788704e-05, -0.038349294531505, 0.9992643473020498]}
}

class ResRo_Manager:
	def __init__(self):
		rospy.init_node('resro_manager')
		# Move_base action client
		self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base_client.wait_for_server()
		rospy.Subscriber('/order_list', StringList, self.order_callback)
		self.cancelService = rospy.Service('/cancel/order', MSGTrigger, self.cancel_service)
		self.current_location = "home"
		self.current_order = None
		self.cancel_order = []
		self.wait_for_confirm = True
		self.not_confirmed_order = []
		self.proceed = True

	def order_callback(self, msg):
		orders = msg.data
		rospy.loginfo("Order received: %s", orders)
		for i,order in enumerate(orders):
			self.wait_for_confirm = True
			if self.current_location == 'home' and self.proceed:
				self.Job_On('kitchen')
				self.wait_for_confirmation("/kitchen_confirmation")
			if self.current_location in ['kitchen','table_1', 'table_2', 'table_3']:
				if order in ['table_1', 'table_2', 'table_3']:
					if order in self.cancel_order:
						rospy.loginfo("order has been cancelled and proceeding further orders...!")
						continue
					self.current_order = order
					self.Job_On(order)
					if self.wait_for_confirm:
						self.wait_for_confirmation("/table_confirmation")
				else:
					rospy.loginfo("Order is invalid")
		if len(self.cancel_order)>0 or len(self.not_confirmed_order)>0:
			self.Job_On('kitchen')
			self.wait_for_confirmation("/kitchen_confirmation")
		self.Job_On('home')


	def wait_for_confirmation(self,topic):
		rospy.loginfo("waiting for %s",topic)
		try:
			rospy.wait_for_message(topic,String,timeout = 15.0)
			rospy.loginfo("confirmation received")
		except rospy.ROSException:
			rospy.loginfo("Confirmation is not received...! proceeding with next job..")
			if self.current_location == 'kitchen':
				self.Job_On('home')
				self.proceed = False
			if self.current_location in ['table_1', 'table_2', 'table_3']:
				self.not_confirmed_order.append(self.current_location)
			
	def Job_On(self, location):
		if location in locations:
			rospy.loginfo("Navigating to %s", location)
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = locations[location]["position"][0]
			goal.target_pose.pose.position.y = locations[location]["position"][1]
			goal.target_pose.pose.orientation.x = locations[location]["orientation"][0]
			goal.target_pose.pose.orientation.y = locations[location]["orientation"][1]
			goal.target_pose.pose.orientation.z = locations[location]["orientation"][2]
			goal.target_pose.pose.orientation.w = locations[location]["orientation"][3]
			self.move_base_client.send_goal(goal)
			self.move_base_client.wait_for_result()
			status = self.move_base_client.get_state()
			if status == 3:
				rospy.loginfo("Reached %s", location)
				self.current_location = location
		else:
			rospy.logwarn("Unknown location: %s", location)
			
	def cancel_service(self,req):
		cancel_req = req.msg
		if cancel_req in ['table_1','table_2','table_3']:
			try:
				print("cancel_request_received :",cancel_req)
				if cancel_req == self.current_order:
					self.cancel_order.append(cancel_req)
					print("cancel_service_enabled")
					self.move_base_client.cancel_all_goals()
					self.wait_for_confirm = False
				else:
					self.cancel_order.append(cancel_req)
				cancel_msg = MSGTriggerResponse()
				cancel_msg.success = True
				cancel_msg.message = 'order has been cancelled successfully..!'
				return cancel_msg

			except Exception as e:
				cancel_msg = MSGTriggerResponse()
				cancel_msg.success = False
				cancel_msg.message = 'cancel service is out of service!'
				return cancel_msg
		else:
			rospy.loginfo("Invalid Cancel Request...!")
	

if __name__ == '__main__':
	try:
		navigator = ResRo_Manager()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Job execution terminated.")

