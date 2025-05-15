from panda3d.core import Vec3, Point3
from panda3d.bullet import BulletCapsuleShape, BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletWorld
from panda3d.bullet import BulletSphereShape, BulletConeTwistConstraint, BulletGenericConstraint
from panda3d.bullet import BulletVehicle
from math import pow
import helper
from typing import Literal

# set up literals
_WHEELPOSTYPES = Literal["Front", "Middle", "Corners"]
_CONTROLTYPES = Literal["Manual", "SpaceStart", "Demo"]


# the robot class is an extension of a bullet vehicle with a list of connected nodes and methods of how to control them 
# (these will be added in the final code)
class Robot(BulletVehicle):
    
    def __init__(self, vehicle, parts_on_init: set):
        parts = parts_on_init

    
    def brakeWheels(self):
        for i in self.getWheels():
            i.setBrake1
    
    def releaseWheels(self):
        for i in self.getWheels():
            i.setBrake(0)
    
    # a method that switches between types of control, demo means the robot moves on a square shaped path
    def setControlType(type: _CONTROLTYPES):
        if type == 'Demo':
            ...




    def moveStart(self, engine_force: float):
        ...

    def moveEnd(self):
        ...


# a function that creates a robot with two nodes and a specified configuration of wheels
def createTwoStoryRobot(world: BulletWorld, base_size: Vec3, upper_size: Vec3, dist: float, wheel_pos: _WHEELPOSTYPES, 
                        position=Vec3(0, 0, 10)) -> Robot:
    
    parts = []

    # create the first box ('base') 
    shape = BulletBoxShape(base_size)
    base_node = BulletRigidBodyNode("Base")
    base_node.setMass((base_size.x + base_size.y + base_size.z))
    base_node.addShape(shape)
    np = render.attachNewNode(base_node)
    np.setPos(position)
    world.attachRigidBody(base_node)
    model = loader.loadModel('models/box.egg')
    model.flattenLight()
    model.reparentTo(np)
    model.setScale(base_size.__mul__(2))
    model.setPos(base_size.__mul__(-1))
    texture = loader.loadTexture('Blue_Fill.png')
    model.setTexture(texture, 1)

    base = base_node

    vehicle = BulletVehicle(world, base)
    world.attachVehicle(vehicle)

    # add another small box and connect it to the first one
    shape = BulletBoxShape(upper_size)
    node = BulletRigidBodyNode("Dongle")
    node.setMass((upper_size.x + upper_size.y + upper_size.z)/3)
    node.addShape(shape)
    np = render.attachNewNode(node)
    position.addZ(dist)
    np.setPos(position)
    world.attachRigidBody(node)
    model = loader.loadModel('models/box.egg')
    model.flattenLight()
    model.reparentTo(np)
    model.setScale(upper_size.__mul__(2))
    model.setPos(upper_size.__mul__(-1))
    texture = loader.loadTexture('Red_Fill.jpg')
    model.setTexture(texture, 1)

    
    cons = helper.connectNodesVertical(world, base, node, dist)

    parts.append([node, cons])

    # add wheels
    if wheel_pos == 'Front':
        for i in range(2):
            wheel = vehicle.createWheel()


            wheel.setChassisConnectionPointCs(Vec3(base_size.getX()*pow(-1, i), -base_size.getY(), 0))
            wheel.setWheelRadius(base_size.getZ()*1.5)
            wheel.setWheelDirectionCs(Vec3(0, 0, -1))
            wheel.setWheelAxleCs(Vec3(pow(-1, i), 0, 0))
            wheel.setMaxSuspensionTravelCm(base_size.getZ()*40) #important

            wheel.setSuspensionStiffness(base_size.getZ()*40) # important
            wheel.setWheelsDampingRelaxation(2.3)
            wheel.setWheelsDampingCompression(4.4)
            wheel.setFrictionSlip(100.0)
            wheel.setRollInfluence(0.1)
            wheel.setFrontWheel(True)
            wheel.setBrake(1)



        wheel = vehicle.createWheel()


        wheel.setChassisConnectionPointCs(Vec3(0 , base_size.getY() + 0.1, 0))
        wheel.setWheelRadius(base_size.getZ())
        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setMaxSuspensionTravelCm(base_size.getZ()*40) #important
        wheel.setSuspensionStiffness(80) # important
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(50)
        wheel.setRollInfluence(0)

    
    if wheel_pos == 'Middle':
        ...
    
    if wheel_pos == 'Corners':
        ...
    
    robot = Robot(vehicle, parts)
    # robot.brakeWheels()
    return robot

