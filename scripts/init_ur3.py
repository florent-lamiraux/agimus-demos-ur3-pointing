import sys, argparse, numpy as np, time, rospy
from math import pi, sqrt
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, Rule, CorbaClient
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator
from utils import (
    Kapla,
    EulerToQuaternion
)

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of UR3 pointing')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
args = p.parse_args ()

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in jointBounds[which]:
        robot.setJointBounds(jn, bound)

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/ur3/pointing/urdf/robot.urdf.xacro",
       "transmission_hw_interface:=hardware_interface/PositionJointInterface")
Robot.srdfString = ""

loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
#client.basic._tools.deleteAllServants()
client.manipulation.problem.selectProblem (args.context)
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)

robot = Robot("robot", "ur3e", rootJointType="anchor", client=client)
crobot = wd(wd(robot.hppcorba.problem.getProblem()).robot())

print("Robot loaded")
robot.opticalFrame = 'camera_color_optical_frame'
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")

ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)


#Viewer Factory
vf = ViewerFactory(ps)
## Shrink joint bounds of UR-5
#
jointBounds = dict()
jointBounds["limited"] = [('ur3e/shoulder_pan_joint', [-pi, pi]),
  ('ur3e/shoulder_lift_joint', [-pi, pi]),
  ('ur3e/elbow_joint', [-3.1, 3.1]),
  ('ur3e/wrist_1_joint', [-3.2, 3.2]),
  ('ur3e/wrist_2_joint', [-3.2, 3.2]),
  ('ur3e/wrist_3_joint', [-3.2, 3.2])]

setRobotJointBounds("limited")

## Remove some collision pairs
#
ur3JointNames = list(filter(lambda j: j.startswith("ur3/"), robot.jointNames))
ur3LinkNames = [ robot.getLinkNames(j) for j in ur3JointNames ]

############
### Part ###
############

#Get class_name str from rosparam
Part_name = rospy.get_param('/demo/objects/kapla/class_name'); #Kapla

# Instanciate the Part
try:
    class_ = globals()[Part_name]
except Exception as e:
    raise ValueError("Probably unvalid part name, check yaml files or rosparam") from e
Part = class_()

vf.loadRobotModel (Part, "kapla")

#Kapla joint
robot.setJointBounds('kapla/root_joint', [-0.388, 0.372,
                                          -0.795, 0.135, 
                                           1.008, 1.7392])
print(f"{Part.__class__.__name__} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur3e", "package://agimus_demos/srdf/ur3_robot.srdf")

#Pose Kapla
qw, qx, qy, qz = EulerToQuaternion(0,0,0)
partPose = [0.1, -0.4, 1.009,qx,qy,qz, qw]

## Define initial configuration
q0 = robot.getCurrentConfig()
r = robot.rankInConfiguration['kapla/root_joint']
q0[r:r+7] = partPose

#Init
v = vf.createViewer()
pp = PathPlayer(v)
ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)

