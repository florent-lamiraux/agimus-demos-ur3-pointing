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
    Pokeball,
    PartPlaque,
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
vf.loadEnvironmentModel(Pokeball, "pokeball")
robot.setJointBounds(
        "pokeball/root_joint",
        [
                -0.4,
                0.4,
                -0.4,
                0.4,
                -0.1,
                2.0,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001,
                -1.0001,
                1.0001],)

## Shrink joint bounds of UR-5
#
jointBounds = dict()
jointBounds["default"] = [ (jn, robot.getJointBounds(jn)) \
                           if not jn.startswith('ur3/') else
                           (jn, [-pi, pi]) for jn in robot.jointNames]
jointBounds["limited"] = [('ur3e/shoulder_pan_joint', [-pi, pi]),
  ('ur3e/shoulder_lift_joint', [-pi, pi]),
  ('ur3e/elbow_joint', [-3.1, 3.1]),
  ('ur3e/wrist_1_joint', [-3.2, 3.2]),
  ('ur3e/wrist_2_joint', [-3.2, 3.2]),
  ('ur3e/wrist_3_joint', [-3.2, 3.2])]
# Bounds to generate calibration configurations
jointBounds["calibration"] = [('ur3e/shoulder_pan_joint', [-2.5, 2.5]),
  ('ur3e/shoulder_lift_joint', [-2.5, 2.5]),
  ('ur3e/elbow_joint', [-2.5, 2.5]),
  ('ur3e/wrist_1_joint', [-2.5, 2.5]),
  ('ur3e/wrist_2_joint', [-2.5, 2.5]),
  ('ur3e/wrist_3_joint', [-2.5, 2.5])]
setRobotJointBounds("limited")

## Remove some collision pairs
#
ur3JointNames = list(filter(lambda j: j.startswith("ur3/"), robot.jointNames))
ur3LinkNames = [ robot.getLinkNames(j) for j in ur3JointNames ]

# Get class_name str from rosparam
Part_name = rospy.get_param('/demo/objects/part/class_name');
# Instanciate the Part
try:
    class_ = globals()[Part_name]
except Exception as e:
    raise ValueError("Probably unvalid part name, check yaml files or rosparam") from e
Part = class_()

vf.loadRobotModel (Part, "part")

# JESSY 07/12 change part/root_joint y: 1.5-> 1.75
robot.setJointBounds('part/root_joint', [1, 1.75, -0.5, 0.5, -0.5, 0.5])
print(f"{Part.__class__.__name__} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur3e", "package://agimus_demos/srdf/ur3_robot.srdf")

# VISUAL(modification pour voir le visual servoing sur Gazebo)
# JESSY 07/12 change partPose[0]: 1.4 -> 1.6
partPose = [1.6, 0.1, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
r = robot.rankInConfiguration['part/root_joint']
q0[r:r+7] = partPose

gripper = 'ur3e/gripper'
## Create specific constraint for a given handle
#  Rotation is free along x axis.
#  Note that locked part should be added to loop edge.
def createFreeRxConstraintForHandle(handle):
    name = gripper + ' grasps ' + handle
    handleJoint, jMh = robot.getHandlePositionInJoint(handle)
    gripperJoint, jMg = robot.getGripperPositionInJoint(gripper)
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, jMg, jMh,
         [True, True, True, False, True, True])
    # pregrasp
    shift = 0.13
    M = Transform(jMg)*Transform([shift,0,0,0,0,0,1])
    name = gripper + ' pregrasps ' + handle
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, M.toTuple(), jMh,
         [True, True, True, False, True, True])


v = vf.createViewer()
v(q0)
pp = PathPlayer(v)

"""
#############
### Begin ###
#############
actory, graph = createConstraintGraph()

try:
    v = vf.createViewer()
    v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")





ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)

pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()


##DEMO
q1 = [-0.3264229933368128, -0.7798698584185999, -1.5035503546344202, -0.044666592274801076, -0.06998950639833623, 0.49361300468444824, 1.6, 0.1, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]
q2 = [-0.03687030473817998, 0.045090675354003906, -0.02318889299501592, -1.5115330854998987, -0.0035083929644983414, 1.5308219194412231, 1.6, 0.1, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]

v(q_init)

"""
