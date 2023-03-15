from init_ur3 import robot, ps, v, ri, q_init
from createConstraintGraph import createConstraintGraph, test_edge, test_node
from tools_hpp import RosInterface, PathGenerator
from hpp.gepetto import PathPlayer

#Get graph
factory, graph = createConstraintGraph()

#Set Init
ps.setInitialConfig(q_init)
v(q_init)

#Set Goal
q_goal = q_init[::]
q_goal[6] = -0.1
ps.addGoalConfig(q_goal) #grasp

pp = PathPlayer(v)

pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()

ps.solve()