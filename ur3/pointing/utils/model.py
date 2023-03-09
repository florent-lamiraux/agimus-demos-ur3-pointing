#Pokeball Model
class Pokeball(object):
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/pokeball.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/pokeball.srdf"


#Plaque Model
class PartPlaque:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"
