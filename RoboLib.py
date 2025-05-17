from panda3d.core import Vec3, PandaNode, NodePath, ClockObject
from panda3d.bullet import BulletCapsuleShape, BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletWorld
from panda3d.bullet import BulletSphereShape, BulletConeTwistConstraint, BulletGenericConstraint
from panda3d.bullet import BulletVehicle
from math import pow
import helper
from typing import Literal

# set up literals
_WHEELPOSTYPES = Literal["Front", "Corners"]
_CONTROLTYPES = Literal["Manual", "SpaceStart", "Demo", "None"]


# the robot class is an extension of a bullet vehicle with a list of connected nodes and methods of how to control them 
# (these will be added in the final code)
class Robot():
    
    
    # this init shouldn't be used on its own but rather as a part of other functions as it requires 
    # the parts to be already parts of the scene graph
    def __init__(self, vehicle, parts_on_init: list, name: str):
        self.parts = parts_on_init
        self.node = PandaNode(name)
        self.np = NodePath(self.node)
        self.np.setName(name)
        self.np.reparentTo(render)
        self.vehicle = vehicle
        chassis_np = NodePath(vehicle.chassis)
        chassis_np.reparentTo(self.np)
        self.mode = 'None'
        self.moving = False
        self.clock = ClockObject()
        self.timer = 10000
        self.move_task_name = str('move' + name)

        if  not taskMgr.hasTaskNamed('checkMode'):
           global robots 
           robots = [self]
           def checkMode(task):
               for i in robots:
                    match i.mode:
                        case 'None':
                            i.moveEnd()
                        case 'Demo':
                            i.moveTimed(60, 3)
               return task.cont
           taskMgr.add(checkMode, 'checkMode')
        else:
            robots.append(self)




    
    def brakeWheels(self):
        for i in self.vehicle.getWheels():
            i.setBrake(100)
    
    
    # a method that switches between types of control, demo means the robot moves on a square shaped path
    def setMoveMode(self, type: _CONTROLTYPES):
        self.mode = type


    def moveStart(self, engine_force: float):
        if not self.moving:
            for i in self.vehicle.wheels:
                i.setEngineForce(engine_force)
                i.setBrake(0)
            self.moving = True

    def moveEnd(self):
        if self.moving:
            for i in self.vehicle.wheels:
                i.setEngineForce(0)
                i.setBrake(100)
            self.moving = False
    
    def moveTimed(self, engine_force: float, time: float):
            if taskMgr.hasTaskNamed(self.move_task_name) != True:
                def moveTimedTemp(task):
                    self.moveStart(engine_force)
                    return task.cont
                taskMgr.add(moveTimedTemp, self.move_task_name)
                self.timer = self.clock.long_time + time
            if self.clock.long_time >= self.timer:
                self.setMoveMode('None')
                taskMgr.remove(self.move_task_name)

                
            

            
        

       


# a function that creates a robot with two nodes and a specified configuration of wheels
def createTwoStoryRobot(world: BulletWorld, base_size: Vec3, upper_size: Vec3, dist: float, wheel_pos: _WHEELPOSTYPES, name: str, 
                        position=Vec3(0, 0, 3), base_mass_mult: float = 1) -> Robot:
    
    parts = []

    # create the first box ('base') 
    shape = BulletBoxShape(base_size)
    base_node = BulletRigidBodyNode("Base")
    base_node.setMass((base_size.x + base_size.y + base_size.z)*base_mass_mult)
    base_node.addShape(shape)
    base_np = render.attachNewNode(base_node)
    base_np.setPos(position)
    world.attachRigidBody(base_node)
    model = loader.loadModel('models/box.egg')
    model.flattenLight()
    model.reparentTo(base_np)
    model.setScale(base_size.__mul__(2))
    model.setPos(base_size.__mul__(-1))
    texture = loader.loadTexture('Blue_Fill.png')
    model.setTexture(texture, 1)

    base = base_node

    vehicle = BulletVehicle(world, base)
    world.attachVehicle(vehicle)

    # add another small box and connect it to the first one
    shape = BulletBoxShape(upper_size)
    node = BulletRigidBodyNode("Part")
    node.setMass((upper_size.x + upper_size.y + upper_size.z)/3)
    node.addShape(shape)
    np = base_np.attachNewNode(node)
    np.setPos(Vec3(0, 0, dist))
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
        powered_wheels = [0, 1]
        for i in range(2):
            wheel = vehicle.createWheel()


            wheel.setChassisConnectionPointCs(Vec3(base_size.getX()*pow(-1, i), -base_size.getY(), 0))
            wheel.setWheelRadius(base_size.getZ()*1.3)
            wheel.setWheelDirectionCs(Vec3(0, 0, -1))
            wheel.setWheelAxleCs(Vec3(-pow(-1, i), 0, 0))
            wheel.setMaxSuspensionTravelCm(base_size.getZ()*60) #important

            wheel.setSuspensionStiffness(base_size.getZ()*20) # important
            wheel.setWheelsDampingRelaxation(2.3)
            wheel.setWheelsDampingCompression(4.4)
            wheel.setFrictionSlip(20)
            wheel.setRollInfluence(0.1)
            wheel.setFrontWheel(True)



        wheel = vehicle.createWheel()


        wheel.setChassisConnectionPointCs(Vec3(0 , base_size.getY() + 0.1, 0))
        wheel.setWheelRadius(base_size.getZ())
        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(-1, 0, 0))
        wheel.setMaxSuspensionTravelCm(base_size.getZ()*40) #important
        wheel.setSuspensionStiffness(80) # important
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(100)
        wheel.setRollInfluence(0)

    
    
    if wheel_pos == 'Corners':
        corners = [Vec3(base_size.getX(), base_size.getY(), 0), Vec3(-base_size.getX(), base_size.getY(), 0), 
                   Vec3(-base_size.getX(), -base_size.getY(), 0), Vec3(base_size.getX(), -base_size.getY(), 0)]
        for i in range(4):
            wheel = vehicle.createWheel()


            wheel.setChassisConnectionPointCs(corners[i])
            wheel.setWheelRadius(base_size.getZ()*1.3)
            wheel.setWheelDirectionCs(Vec3(0, 0, -1))
            wheel.setWheelAxleCs(Vec3(-corners[i].normalized()))
            wheel.setMaxSuspensionTravelCm(base_size.getZ()*60) #important

            wheel.setSuspensionStiffness(base_size.getZ()*20) # important
            wheel.setWheelsDampingRelaxation(2.3)
            wheel.setWheelsDampingCompression(4.4)
            wheel.setFrictionSlip(20)
            wheel.setRollInfluence(0.1)
            wheel.setFrontWheel(False)
    
    robot = Robot(vehicle, parts, name)
    robot.moveEnd()
    # robot.brakeWheels()
    return robot

