#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import openai
import json
import tf.transformations
from tf.transformations import quaternion_from_euler
from math import pi
import ast
import numpy as np

openai.api_key = ""
model_id="ft:gpt-3.5-turbo-1106:personal::8Mw1RT3j"
print("Model name: ", model_id)
function_descriptions = [
	{
		"name": "move_group",
		"description": "Gives the transalational(metres) movements and angular(radians) rotations in x,y and z axis.",
		"parameters": {
			"type": "object",
			"properties": {
				"x": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Movement along x axis, the movement can be in +x axis(right) or in -x axis(left)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the x-axis movement is relative"
							}
						}
					},
				"y": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Movement along y axis, the movement can be in +y axis(forward) or in -y axis(backward)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the y-axis movement is relative"
							}
						}
					},
				"z": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Movement along z axis, the movement can be in +z axis(up) or in -z axis(down)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the z-axis movement is relative"
							}
						}
					},
				"roll": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Rotation along x-axis(longitudinal axis), the movement can be in +x axis(positive) or in -x axis(negative)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the rotation in x-axis is relative"
							}
						}
					},
				"pitch": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Rotation along y-axis(lateral axis), the movement can be in +y axis(positive/front) or in -y axis(negative/back)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the rotation in y-axis is relative"
							}
						}
					},
				"yaw": {
					"type": "object",
					"properties": {
						"value":{
							"type": "integer",
							"description": "Rotation along z-axis(vertical axis), the movement can be in clockwise(-ve) or anti-clockwise(+ve)"
							},
						"is_relative":{
							"type": "boolean",
							"description": "Specifies if the rotation in z-axis is relative"
							}
						}
					},
				}
			}
		}
]

def ask_function_calling(query):
	messages = [{"role": "user", "content":"Go to x=5 and yaw to the right"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"x":{"value":5,"is_relative":"false"}, "yaw":{"value":1.745,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"Go right"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"x":{"value":0.05,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"Go left"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"x":{"value":-0.05,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"Go front"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"y":{"value":0.05,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"Go back"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"y":{"value":-0.05,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"Turn  slightly left"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"x":{"value":-0.025,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	{"role": "user", "content":"take right and Go slightly front"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"x":{"value":0.05,"is_relative":"true"},"y":{"value":0.025,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	
	{"role": "user", "content": query}]
	
	response = openai.ChatCompletion.create(
			model=model_id,
			messages=messages,
			temperature=0.1,
			max_tokens=100,
			top_p=1.0,
			functions=function_descriptions,
			function_call="auto"
			)

	#print(response["choices"][0]["message"])
	
	function_name=response["choices"][0]["message"]["function_call"]
	function_arguments_str = response["choices"][0]["message"]["function_call"]["arguments"]
	function_arguments_dict = eval(function_arguments_str)
		
	function_arguments_dict = eval(function_arguments_str)

	for arg, val in function_arguments_dict.items():
		x_value = function_arguments_dict.get("x", {"value": 0.0, "is_relative": False})
		y_value = function_arguments_dict.get("y", {"value": 0.0, "is_relative": False})
		z_value = function_arguments_dict.get("z", {"value": 0.0, "is_relative": False})
		roll_value = function_arguments_dict.get("roll", {"value": 0.0, "is_relative": False})
		pitch_value = function_arguments_dict.get("pitch", {"value": 0.0, "is_relative": False})
		yaw_value = function_arguments_dict.get("yaw", {"value": 0.0, "is_relative": False})


	return function_arguments_dict
	
if __name__ == "__main__":

	try:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
		end_effector_link = "panda_link8"
		robot = moveit_commander.RobotCommander()
		group_name = "panda_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		move_group.set_goal_tolerance(0.0001)
		print("Movegroup successfull")
		
		
		current_pose = move_group.get_current_pose().pose
		current_posx = current_pose.position.x
		current_posy = current_pose.position.y
		current_posz = current_pose.position.z
		current_posrx = current_pose.orientation.x
		current_posry = current_pose.orientation.y
		current_posrz = current_pose.orientation.z
		current_posrw = current_pose.orientation.w

		print("Current position is:", current_pose)
		
		#qw, qx, qy, qz = quaternion_from_euler(0, pi, 0)
		
		
		gpt=ask_function_calling(input("\nGive the command:\n"))
		print(gpt)
		destination_posx, destination_posy, destination_posz,destination_posa = (current_posx, current_posy, current_posz, [0.0, 0.0, 0.0, 0.0])
		for i in gpt:
			print("----------------------")
			if gpt[i]['is_relative'] == 'false':
    				gpt[i]['is_relative'] = False
			else:
    				gpt[i]['is_relative'] = True

			if i=='x' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				print("hii")
				destination_posx = gpt[i]['value']
				print("X is absolute here")
			elif i=='x' and gpt[i]['value'] != 0.0 and gpt[i]['is_relative']:
				destination_posx = current_pose.position.x + gpt[i]['value']
				print("X is relative here")
			
			if i=='y' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				destination_posy = gpt[i]['value']
				print("Y is absolute here")
			elif i=='y' and gpt[i]['value'] != 0.0 and gpt[i]['is_relative']:
				destination_posy = current_pose.position.y +gpt[i]['value']
				print("Y is relative here")

			if i=='z' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				destination_posz = gpt[i]['value']
				print("Z is absolute here")
			elif i=='z' and gpt[i]['value'] != 0.0 and gpt[i]['is_relative']:
				destination_posz = current_pose.position.z +gpt[i]['value']
				print("Z is relative here")
				
			if i=='roll' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,1.0]
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
				
					roll = gpt[i]['value']
					q2 = quaternion_from_euler(roll,0,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					print("roll is second absolute here")
				else:
					q1_inv=[0.0,0.0,0.0,1.0]
				
					roll = gpt[i]['value']
					q2 = quaternion_from_euler(roll,0,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
					print("roll is absolute here")
			elif i=='roll' and gpt[i]['value'] != 0.0 and gpt[i]['is_relative']:
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
				
					roll = gpt[i]['value']
					q2 = quaternion_from_euler(0,roll,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					print(destination_posa)
					print("roll is second relative here")
				else:
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
				
					roll = gpt[i]['value']
					q2 = quaternion_from_euler(0,roll,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
					print(destination_posa)
					print("roll is relative here")
			
			if i=='pitch' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,1.0]
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
				
					pitch = gpt[i]['value']
					q2 = quaternion_from_euler(0,pitch,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					
				else:
					q1_inv=[0.0,0.0,0.0,1.0]
				
					pitch = gpt[i]['value']
					q2 = quaternion_from_euler(0,pitch,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
				print("pitch is absolute here")
			elif i=='pitch' and gpt[i]['value'] != 0.0 and  gpt[i]['is_relative']:
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
				
					pitch = gpt[i]['value']
					q2 = quaternion_from_euler(0,pitch,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					print(destination_posa)
					print("pitch is second relative here")
				else:
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
				
					pitch = gpt[i]['value']
					q2 = quaternion_from_euler(0,pitch,0)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
					print(destination_posa)
					print("pitch is relative here")
				
			if i=='yaw' and gpt[i]['value'] != 0.0 and not gpt[i]['is_relative']:
				print(destination_posa)
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,1.0]
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
				
					yaw = gpt[i]['value']
					q2 = quaternion_from_euler(0,0,yaw)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					print("yaw is second absolute here")
				else:
					q1_inv=[0.0,0.0,0.0,1.0]
				
					yaw = gpt[i]['value']
					q2 = quaternion_from_euler(0,0,yaw)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
					print("yaw is absolute here")
			elif i=='yaw' and gpt[i]['value'] != 0.0 and gpt[i]['is_relative']:
				if not np.array_equal(destination_posa, np.array([0.0, 0.0, 0.0, 0.0])):
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
					q2_inv=tf.transformations.quaternion_multiply(destination_posa, q1_inv)
					print(destination_posa)
					print("------")
					print(q2_inv)
				
					yaw = gpt[i]['value']
					q2 = quaternion_from_euler(0,0,yaw)
					destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
					print(destination_posa)
					print("yaw is second relative here")
				else:
					q1_inv=[0.0,0.0,0.0,0.0]
					q1_inv[0] = current_pose.orientation.x
					q1_inv[1] = current_pose.orientation.y
					q1_inv[2] = current_pose.orientation.z
					q1_inv[3] = -current_pose.orientation.w # Negate for inverse
					
				
					yaw = gpt[i]['value']
					q2 = quaternion_from_euler(0,0,yaw)
					destination_posa = tf.transformations.quaternion_multiply(q2, q1_inv)
					#print(destination_posa)
					print("yaw is relative here")
				
		pose_goal = geometry_msgs.msg.Pose()

		pose_goal.orientation.x = destination_posa[0]
		pose_goal.orientation.y = destination_posa[1]
		pose_goal.orientation.z = destination_posa[2]
		pose_goal.orientation.w = destination_posa[3]
		pose_goal.position.x = destination_posx
		pose_goal.position.y = destination_posy
		pose_goal.position.z = destination_posz
		
		#print(pose_goal)
		print("----------------------")
		move_group.set_pose_target(pose_goal,end_effector_link)
		
		
		move_group.set_num_planning_attempts(3)
		move_group.set_max_velocity_scaling_factor(0.08)
		plan = move_group.go(wait=True)
		print("Planning and executing movement to the specified destination...")
		
		print(move_group.get_current_pose().pose)
		
		if plan:
			print("Movement to the specified destination successful!")
		else:
			print("Failed to plan the movement to the specified destination.")
			
		move_group.stop()

		# Clear targets
		move_group.clear_pose_targets()

	except rospy.ROSInterruptException:
		print("Movegroup failure")
