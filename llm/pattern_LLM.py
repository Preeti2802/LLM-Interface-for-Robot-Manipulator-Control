#!/usr/bin/env python3

import openai
import json
import math
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import tf.transformations
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi
import ast
import numpy as np

openai.api_key = ""
model_id = "ft:gpt-3.5-turbo-0613:personal::8LvIacI0"
#model_id='ft:gpt-3.5-turbo-0613:personal::8cmP5OA3'
function_descriptions = [
    {
        "name": "move_group",
        "description": "Gives the translational (meters) movements and angular (radians) rotations in x, y, and z axis.",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Movement along x axis, the movement can be in +x axis (right) or in -x axis (left). For diagonal or angled movements such as 'back-right', the x-component is calculated as the cosine of the movement angle times the movement."
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the x-axis movement is relative"
                        }
                    }
                    },
                "y": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Movement along y axis, the movement can be in +y axis (forward) or in -y axis (backward). For diagonal or angled movements such as 'front-right', the x-component is calculated as the sine of the movement angle times the movement."
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the y-axis movement is relative"
                        }
                    }
                },
                "z": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Movement along z axis, the movement can be in +z axis (up) or in -z axis (down)"
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the z-axis movement is relative"
                        }
                    }
                },
                "roll": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Rotation along x-axis (longitudinal axis), the movement can be in +x axis (positive) or in -x axis (negative)"
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the rotation in x-axis is relative"
                        }
                    }
                },
                "pitch": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Rotation along y-axis (lateral axis), the movement can be in +y axis (positive/front) or in -y axis (negative/back)"
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the rotation in y-axis is relative"
                        }
                    }
                },
                "yaw": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Rotation along z-axis (vertical axis), the movement can be in clockwise (-ve) or anti-clockwise (+ve)"
                        },
                        "is_relative": {
                            "type": "boolean",
                            "description": "Specifies if the rotation in z-axis is relative"
                        }
                    }
                }
            }
        }
    },
    {
        "name": "move_pattern",
        "description": "Covers movement in desired pattern path",
        "parameters": {
            "type": "object",
            "properties": {
                "pattern_name": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "string",
                            "description": "Gives name of the pattern shape mentioned in the query. For example: If it is square then it is quadrilateral",
                        },
                    }
                },
                "side_length": {
                    "type": "object",
                    "properties": {
                        "value": {
                            "type": "integer",
                            "description": "Gives length of each side as described in the query. Units is metres."
                        },
                    }
                }
            }
        }
    }
]

def generating_pattern_commands(pattern_name, side_length):

    query = f"Generate a sequence of movement commands for regular closed {pattern_name} in a 2D frame. Include movements commands like front-left, front-right,back-left, back-right, back and front when necessary as the reference frame remains fixed. Each side movement includes {side_length}m."


    messages = [{"role": "system", "content": "You are a helpful assistant that provides movement commands."},
                {"role": "user", "content": "Generate a sequence of movement commands for regular closed square in a 2D frame. Include movements commands like front-left, front-right,back-left, back-right, back and front when necessary as the reference frame remains fixed. Each side movement includes 0.2m."},
                {"role": "assistant", "content": "1. Move 0.2 front with an relative yaw angle of 90 degrees\n2. move 0.2 left with an relative yaw angle of 90 degrees\n3. Move 0.2 back with an relative yaw angle of 90 degrees\n4. move 0.2 right with an relative yaw angle of 90 degrees"},
                {"role": "user", "content":"Generate a sequence of movement commands for regular closed pentagon in a 2D frame. Include movements commands like front-left, front-right,back-left, back-right, back and front when necessary as the reference frame remains fixed. Each side movementincludes 0.1m."},
                {"role": "assistant", "content":"1. Move 0.1m front-right with an relative yaw angle of 72 degrees\n2. Move 0.1m front with an relative yaw angle of 72 degrees \n3. Move 0.1m front-left with an relative yaw angle of 72 degrees \n4. Move 0.1m back-left with an relative yaw angle of 72 degrees \n5. Move 0.1m back-right with an relative yaw angle of 72 degrees"},
                {"role": "user", "content":"Generate a sequence of movement commands for regular closed hexagon in a 2D frame. Include movements commands like front-left, front-right,back-left, back-right, back and front when necessary as the reference frame remains fixed. Each side movement includes 0.3m."},
                {"role": "assistant", "content":"1. Move 0.3m front-right with an relative yaw angle of 60 degrees\n2. Move 0.3m front with an relative yaw angle of 60 degrees\n3. Move 0.3m  front-left with an relative yaw angle of 60 degrees\n4. Move 0.3m  back-left with an relative yaw angle of 60 degrees\n5. Move 0.3m back with an relative yaw angle of 60 degrees\n6. Move 0.3m back-right with an relative yaw angle of 60 degrees"},
                {"role": "user", "content":query}
                ]
    
    response = openai.ChatCompletion.create(
        model='gpt-3.5-turbo-1106',
        messages=messages,
        temperature=0.1,
        max_tokens=250,
        top_p=1.0,
    )

    comm_str=response["choices"][0]["message"]['content']
    commands_list = [' '.join(line.split()[1:]) for line in comm_str.split('\n') if line.strip() and line.strip()[0].isdigit()]

    return commands_list

def user_prompt(query):
    messages = [{"role": "user", "content": query}]
    response = openai.ChatCompletion.create(
        model=model_id,
        messages=messages,
        temperature=0.1,
        max_tokens=100,
        top_p=1.0,
        functions=function_descriptions,
        function_call="auto"
    )

    print("Function call response:", response["choices"][0]["message"]["function_call"])
    if response["choices"][0]["message"]["function_call"]["name"]=='move_pattern':
        # Extract sides and side_length
        arg = response["choices"][0]["message"]["function_call"]["arguments"]
        arg = json.loads(arg)
        pattern_name = arg["pattern_name"]
        print("Pattern shape:", pattern_name)
        side_length = arg["side_length"]
        print("Length of sides:", side_length)

        # Generate and print polygon commands
        polygon_commands = generating_pattern_commands(pattern_name, side_length)
        #polygon_commands=['Move 0.12m right with a relative roll angle of 72 degrees', 'Move 0.12m right with a relative roll angle of 72 degrees', 'Move 0.12m right with a relative roll angle of 72 degrees', 'Move 0.12m right with a relative roll angle of 72 degrees', 'Move 0.12m right with a relative roll angle of 72 degrees']
        print("Polygon commands:", polygon_commands)
        return polygon_commands
        
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
	
	{"role": "user", "content":"Move 0.1m front-left with a relative yaw angle of 72 degrees"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"y":{"value":0.1 * math.sin(math.radians(72)),"is_relative":"true"},"x":{"value":-0.1 * math.cos(math.radians(72)),"is_relative":"true"}, "yaw":{"value":1.256,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	
	{"role": "user", "content":"Move 0.1m back-right with a relative yaw angle of 72 degrees"},
	{'role': 'assistant', 'content': None,'function_call':{"name": function_descriptions[0]["name"], "arguments": json.dumps({"y":{"value":-0.1 * math.sin(math.radians(72))
,"is_relative":"true"},"x":{"value":0.1* math.cos(math.radians(72)),"is_relative":"true"}, "yaw":{"value":1.256,"is_relative":"true"}},indent=2,ensure_ascii=False,)}},
	
	{"role": "user", "content":"Move 0.19m  front-right with a relative yaw angle of 90 degrees"},
	{'role': 'assistant', 'content': None, 'function_call': {"name": function_descriptions[0]["name"], "arguments": json.dumps({"y": {"value": 0.19 * math.sin(math.radians(90)), "is_relative": "true"}, "x": {"value": 0.19 * math.cos(math.radians(90)), "is_relative": "true"}, "yaw": {"value": 1.571, "is_relative": "true"}}, indent=2, ensure_ascii=False)}},
	
	{"role": "user", "content":"Move 0.25m  front-right with a relative yaw angle of 30 degrees"},
	{'role': 'assistant', 'content': None, 'function_call': {"name": function_descriptions[0]["name"], "arguments": json.dumps({"y": {"value": 0.25 * math.sin(math.radians(30)), "is_relative": "true"}, "x": {"value": 0.25 * math.cos(math.radians(30)), "is_relative": "true"}, "yaw": {"value": 0.52359, "is_relative": "true"}}, indent=2, ensure_ascii=False)}},

	
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
	function_arguments_dict = json.loads(function_arguments_str)
		
	for arg, val in function_arguments_dict.items():
		x_value = function_arguments_dict.get("x", {"value": 0.0, "is_relative": False})
		y_value = function_arguments_dict.get("y", {"value": 0.0, "is_relative": False})
		z_value = function_arguments_dict.get("z", {"value": 0.0, "is_relative": False})
		roll_value = function_arguments_dict.get("roll", {"value": 0.0, "is_relative": False})
		pitch_value = function_arguments_dict.get("pitch", {"value": 0.0, "is_relative": False})
		yaw_value = function_arguments_dict.get("yaw", {"value": 0.0, "is_relative": False})


	return function_arguments_dict

# Example call
if __name__ == "__main__":
    try:

        move_patterns_commands = user_prompt(input("Give the user command: "))
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        end_effector_link = "panda_link8"
        robot = moveit_commander.RobotCommander()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_goal_tolerance(0.0001)
        print("Movegroup successful")
            

        for k in move_patterns_commands:
            print(k)
            gpt=ask_function_calling(str(k))
            print(gpt)
            current_pose = move_group.get_current_pose().pose
            current_posx = current_pose.position.x
            current_posy = current_pose.position.y
            current_posz = current_pose.position.z
            current_posrx = current_pose.orientation.x
            current_posry = current_pose.orientation.y
            current_posrz = current_pose.orientation.z
            current_posrw = current_pose.orientation.w
            print("Current position is:", current_pose)

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
                    
                        yaw = gpt[i]['value']
                        q2 = quaternion_from_euler(roll,0,0)
                        destination_posa = tf.transformations.quaternion_multiply(q2, q2_inv)
                        print("roll is second absolute here")
                    else:
                        q1_inv=[0.0,0.0,0.0,1.0]
                    
                        yaw = gpt[i]['value']
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

            move_group.set_num_planning_attempts(6)
            move_group.set_max_velocity_scaling_factor(0.08)
            plan = move_group.go(wait=True)
            print("Planning and executing movement to the specified destination...")
            print(move_group.get_current_pose().pose)

            if plan:
                print("Movement to the specified destination successful!")
            else:
                print("Failed to plan the movement to the specified destination.")
        move_group.stop()
        move_group.clear_pose_targets()
    except rospy.ROSInterruptException:
        pass
