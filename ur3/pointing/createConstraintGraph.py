from numpy import pi
from hpp.corbaserver.manipulation import ConstraintGraph, ConstraintGraphFactory,  Constraints, SecurityMargins
from utils import norm, EulerToQuaternion
def createConstraintGraph(robot, ps):

    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["ur3e/gripper",])
    factory.generate()

    #Constraints
    qw, qx, qy, qz = EulerToQuaternion(-pi/2,3*pi/2,pi)
    print(qw, qx, qy, qz)
        
    ps.createTransformationConstraint(
            'grasp', 
            'ur3e/ee_gripper',
            'kapla/root_joint',
            [0, 0.12, 0.075,qx,qy,qz,qw],
            [True, True, True, True, True, True,],)
    
    #Kapla in horizontal plane with free rotation z
    ps.createTransformationConstraint(
            'placement', 
            '',
            'kapla/root_joint',
            [0, 0, 1.009, 0,0,0,1],
            [False, False, True, True, True, False,],)
    
    ps.createTransformationConstraint(
            'placement/complement', 
            '',
            'kapla/root_joint',
            [0, 0, 1.009, 0,0,0,1],
            [True, True, False, False, False, True],)
    
    ps.createTransformationConstraint(
            'pre-grasp', 
            'ur3e/ee_gripper',
            'kapla/root_joint',
            [0, 0.18, 0.075,qx,qy,qz,qw],
            [True, True, True, True, True, True,],)
     
    ps.createTransformationConstraint(
            'pre-grasp/complement', 
            'ur3e/gripper',
            'kapla/root_joint',
            [0, 0, 0, qx,qy,qz, qw],
            [False, False, False, False, False, False,],)
    ps.createTransformationConstraint(
            'pre-release', 
            '',
            'kapla/root_joint',
            [0, 0, 1.1009, 0,0,0,1],
            [False, False, True, True, True, True,],)

    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("placement/complement", False)
    ps.setConstantRightHandSide("pre-grasp", True)
    ps.setConstantRightHandSide("pre-grasp/complement", False)
    #Nodeskapla/root_joint
    graph.createNode(['grasp_placement',
                      'kapla_above_ground',
                      'gripper_above_kapla',
                      'grasp',
                      'placement'])

    #Edge
    graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
    graph.createEdge('placement', 'gripper_above_kapla', 'approach_kapla', 1, 'placement')
    graph.createEdge('gripper_above_kapla', 'grasp_placement', 'grasp_kapla', 1, 'placement')
    graph.createEdge('grasp_placement', 'kapla_above_ground', 'take_kapla_up', 1, 'grasp')
    graph.createEdge('kapla_above_ground', 'grasp', 'take_kapla_away', 1, 'grasp')
    graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
    graph.createEdge('grasp', 'kapla_above_ground', 'approach_ground', 1, 'grasp')
    graph.createEdge('kapla_above_ground', 'grasp_placement', 'put_kapla_down', 1, 'grasp')
    graph.createEdge('grasp_placement', 'gripper_above_kapla', 'move_gripper_up', 1, 'placement')
    graph.createEdge('gripper_above_kapla', 'placement', 'move_gripper_away', 1, 'placement')

    #Apply constraints
    graph.addConstraints(node='placement',constraints = Constraints(numConstraints=['placement']))
    graph.addConstraints(node='gripper_above_kapla',constraints = Constraints(numConstraints=['pre-grasp', 'placement']))
    graph.addConstraints(node='grasp_placement',constraints = Constraints(numConstraints=['placement', 'grasp']))
    graph.addConstraints(node='kapla_above_ground',constraints = Constraints(numConstraints=['pre-release','grasp']))
    graph.addConstraints(node='grasp',constraints = Constraints(numConstraints=['grasp']))
    
    graph.addConstraints(edge='transit', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='approach_kapla', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='grasp_kapla', constraints = Constraints(numConstraints=['placement/complement','pre-grasp/complement']))
    graph.addConstraints(edge='take_kapla_up', constraints = Constraints(numConstraints=['placement/complement','grasp']))
    graph.addConstraints(edge='take_kapla_away', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='transfer', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='approach_ground', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='put_kapla_down', constraints = Constraints(numConstraints=['grasp', 'placement/complement']))
    graph.addConstraints(edge='move_gripper_up', constraints = Constraints(numConstraints=['pre-grasp/complement', 'placement/complement']))
    graph.addConstraints(edge='move_gripper_away', constraints = Constraints(numConstraints=[ 'placement/complement']))

    #Security Margin
    sm = SecurityMargins(ps, factory, ["ur3e", "kapla"])
    sm.setSecurityMarginBetween("ur3e", "ur3e", 0)
    sm.defaultMargin = 0.01
    sm.apply()
    graph.initialize()
    
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph


def createConstraintGraphPointing(robot, ps):
    # Return a list of available elements of type type handle
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
  #  sm = SecurityMargins(ps, factory, ["ur3e", "part"])
 #   sm.setSecurityMarginBetween("ur3e", "part", 0.015)
 #   sm.setSecurityMarginBetween("ur3e", "ur3e", 0)
 #   sm.defaultMargin = 0.01
 #   sm.apply()
    graph.initialize()
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph

def test_node(node, graph, robot):
    for i in range(100):
        q = robot.shootRandomConfig()
        res, q1, err = graph.applyNodeConstraints(node, q)
        if res:
            res, err = robot.isConfigValid(q1)
            if res: break
    return res, q1, err    
    
def test_edge(edge, q_i, graph, robot):
    for i in range(100):
        q = robot.shootRandomConfig()
        res, q1, err = graph.generateTargetConfig(edge,q_i, q)
        #v(q1)
        if res:
            res, err = robot.isConfigValid(q1)
            if res: break
    return res, q1, err   




