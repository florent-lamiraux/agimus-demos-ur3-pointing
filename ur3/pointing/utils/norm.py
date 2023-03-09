from math import sqrt

def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))
