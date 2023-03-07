import sys, argparse, numpy as np, time, rospy
from math import pi, sqrt
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator

def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

#Plaque Model
class PartPlaque:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"

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
# ps.selectConfigurationShooter('Gaussian')
# ps.setParameter('ConfigurationShooter/Gaussian/center', 12*[0.] + [1.])
# ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.25)
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)

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

## Build constraint graph
def createConstraintGraph():
    all_handles = ps.getAvailable('handle')
    part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))

    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["ur3e/gripper",])
    factory.setObjects(["part",], [part_handles], [[]])
    factory.generate()
    for handle in all_handles:
        loopEdge = 'Loop | 0-{}'.format(factory.handles.index(handle))
        graph.addConstraints(edge = loopEdge, constraints = Constraints
            (numConstraints=['part/root_joint']))

    n = norm([-0.576, -0.002, 0.025, 0.817])
    ps.createTransformationConstraint('look-at-part', 'part/base_link', 'ur3e/wrist_3_link',
                                    [-0.126, -0.611, 1.209, -0.576/n, -0.002/n, 0.025/n, 0.817/n],
                                    [True, True, True, True, True, True,])
    graph.createNode(['look-at-part'])
    graph.createEdge('free', 'look-at-part', 'go-look-at-part', 1, 'free')
    graph.createEdge('look-at-part', 'free', 'stop-looking-at-part', 1, 'free')

    graph.addConstraints(node='look-at-part',
                        constraints = Constraints(numConstraints=['look-at-part']))
    ps.createTransformationConstraint('placement/complement', '','part/base_link',
                                    [0,0,0,0, 0, 0, 1],
                                    [True, True, True, True, True, True,])
    ps.setConstantRightHandSide('placement/complement', False)
    graph.addConstraints(edge='go-look-at-part',
                        constraints = Constraints(numConstraints=\
                                                ['placement/complement']))
    graph.addConstraints(edge='stop-looking-at-part',
                        constraints = Constraints(numConstraints=\
                                                ['placement/complement']))
    sm = SecurityMargins(ps, factory, ["ur3e", "part"])
    sm.setSecurityMarginBetween("ur3e", "part", 0.015)
    sm.setSecurityMarginBetween("ur3e", "ur3e", 0)
    sm.defaultMargin = 0.01
    sm.apply()
    graph.initialize()
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph

factory, graph = createConstraintGraph()

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

v(q_init)

pg.demo_planAndExecute(q1, q_init)
