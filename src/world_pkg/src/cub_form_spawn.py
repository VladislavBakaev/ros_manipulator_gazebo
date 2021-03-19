#!/usr/bin/env python3
import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import tf

rospy.wait_for_service('gazebo/spawn_urdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
cubs_status = False

def init_polygon():
    global cubs_status
    pose_red = Pose()
    form_pose = Pose()

    pose_red.position.z = 0.10
    pose_red.position.x = -0.235
    pose_red.position.y = 0.231

    form_pose.position.x = -0.19
    form_pose.position.z = 0.1
    form_pose.position.y = -0.231

    angles = tf.transformations.quaternion_from_euler(0, 0, 1.57)
    form_pose.orientation.x = angles[0]
    form_pose.orientation.y = angles[1]
    form_pose.orientation.z = angles[2]
    form_pose.orientation.w = angles[3]



    cub_model_f = open(cub_model_path,'r')
    cub_model = cub_model_f.read()

    form_model_f = open(form_model_path, 'r')
    form_model = form_model_f.read()

    if(not cubs_status):
        red_cub_model = cub_model.replace('Color','Red')
        spawn_model_prox('cub_red', red_cub_model, "object", pose_red, "world")

        spawn_model_prox('form', form_model, 'object', form_pose, "world")
        cubs_status = True
    

def restart_polygon(msg):
    global cubs_status
    if(cubs_status):
        delete_cubs('')
        cubs_status = False
    init_polygon()
    return msg

def delete_cubs(msg):
    global cubs_status
    if(cubs_status):
        del_model_prox('cub_red')
        del_model_prox('form')
        cubs_status = False
    return msg

if __name__=='__main__':
    global cub_model_path, form_model_path
    rospy.init_node('cube_spawn')

    model_dir = os.path.dirname(os.path.realpath(__file__))
    model_dir = model_dir[0:-3]
    model_dir = model_dir + 'urdf/'
    cub_model_path = model_dir+'cubs.urdf'
    form_model_path = model_dir+'form.urdf'
    init_polygon()
    rospy.Service('/polygon/restart_cubs',Empty,restart_polygon)
    rospy.Service('/polygon/delete_cubs',Empty,delete_cubs)
    rospy.spin()
