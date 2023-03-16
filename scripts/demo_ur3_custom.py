from init_ur3 import robot, ps, v, ri, q_init
from createConstraintGraph import createConstraintGraphCustom, test_edge, test_node
from tools_hpp import RosInterface, PathGenerator
from hpp.gepetto import PathPlayer

#get graph
factory, graph = createConstraintGraphCustom()

"""
i = 0
p = 0
res, q, err = test_node('grasp', graph, robot)
print(res, err)
v(q)
#Test graph pas à pas
while(i<1):
    i +=1
    res, q1, err = test_edge('transit', q_init, graph, robot)
    p = 1
    v(q1)
    if not res: continue
    res, q2, err = test_edge('approach_kapla', q_init, graph, robot)
    p = 2
    v(q2)
    if not res: continue
    res, q3, err = test_edge('grasp_kapla', q2, graph, robot)
    p = 3
    v(q3)
    if not res: continue
    res, q4, err = test_edge('take_kapla_up', q3, graph, robot)
    p = 4
    v(q4)
    if not res: continue
    res, q5, err = test_edge('take_kapla_away', q4, graph, robot)
    p = 5
    v(q5)
    if not res: continue
    res, q6, err = test_edge('approach_ground', q5, graph, robot)
    p = 6
    v(q6)
    if not res: continue
    res, q7, err = test_edge('put_kapla_down', q6, graph, robot)
    p = 7
    v(q7)
    if not res: continue
    res, q8, err = test_edge('move_gripper_up', q7, graph, robot)
    p = 8
    v(q8)
    if not res: continue
    res, q9, err = test_edge('move_gripper_away', q8, graph, robot)
    p = 9
    v(q9)
    if not res: continue
print(res, err, p)
"""

#Set Init
ps.setInitialConfig(q_init)
v(q_init)

#Set Goal
q_goal = q_init[::]
q_goal[6] = -0.1
ps.addGoalConfig(q_goal) #grasp

pp = PathPlayer(v)

pg = PathGenerator(ps, graph, ri, v, q_init)
pg.testGraph()

ps.solve()