#Pokeball Model
class Pokeball(object):
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/pokeball.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/pokeball.srdf"

#Kapla Model
class Kapla(object):
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/kapla.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/kapla.srdf"
    
#Plaque Model
class PartPlaque:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"
