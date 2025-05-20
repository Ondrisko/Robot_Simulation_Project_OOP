from panda3d.core import Vec3, PandaNode, NodePath, ClockObject, Vec2
from panda3d.bullet import BulletCapsuleShape, BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletWorld
from panda3d.bullet import BulletSphereShape, BulletConeTwistConstraint, BulletGenericConstraint
from panda3d.bullet import BulletVehicle
from direct.showbase import DirectObject
from math import pow, sin, cos, radians
import helper
from typing import Literal

# set up literals
_WHEELPOSTYPES = Literal["Front", "Corners", "OneWheel"]
_CONTROLTYPES = Literal["SpaceStart", "Demo", "None", "Coast", "Forward", "PID_Demo"]


# the robot class is an extension of a bullet vehicle with a list of connected nodes and methods of how to control them 
# (these will be added in the final code)
class Robot(DirectObject.DirectObject):
    
    
    # this init shouldn't be used on its own but rather as a part of other functions as it requires 
    # the parts to be already parts of the scene graph
    def __init__(self, vehicle, parts_on_init: list, powered_wheels: list, name: str, PID):
        # the parts of the robot and wheels into which force should go
        self.parts = parts_on_init
        self.powered_wheels = powered_wheels
        
        # add various parts for the robot to be able to interact with phe panda3d framework
        self.node = PandaNode(name)

        self.np = NodePath(self.node)
        self.np.setName(name)
        self.np.reparentTo(render)

        self.vehicle = vehicle

        self.chassis_np = NodePath(vehicle.chassis)
        self.chassis_np.reparentTo(self.np)
        
        # setup movement mode, clock, set timer as very large number, setup names for tasks that move and turn the robot
        self.mode = 'None'
        self.moving = False
        self.clock = ClockObject()
        self.timer = 1000000000
        self.move_task_name = str('move' + name)
        self.turn_task_name = str('turn' + name)
        
        # setup default force for robots movement dependant on its mass, reset stage of the demo and initialize 
        # target direction for the PID and turning
        self.default_force = vehicle.chassis.mass*5
        self.move_demo_stage = 0
        target_direction = self.vehicle.getForwardVector()
        self.target_direction = target_direction.xy

        # add a functionality to start the robots movement with space using panda event listener
        self.accept('space', self.spaceStart)
        
        # setup PID
        self.PID = PID
        
        # if this is the first robot created, create a task for the task manager 
        # which checks and moves all robots form a global list
        if  not taskMgr.hasTaskNamed('checkMode'):
           global robots 
           robots = [self]
           def checkMode(task):
               for i in robots:
                    match i.mode:
                        case 'None':
                            i.moveEnd()
                        case 'Demo':
                            i.moveDemoFrontWheelRobot()
                        case 'Coast':
                            i.moveStart(0)
                        case 'SpaceStart':
                            if i.space_start:
                                i.setMoveMode('Demo')
                        case 'Forward':
                            i.moveStart(self.default_force)
            
               return task.cont
           taskMgr.add(checkMode, 'checkMode')
        
        # otherwise just add the robot to the list
        else:
            robots.append(self)


    # a method that changes a value once space is pressed
    def spaceStart(self):
        self.space_start = True

    # a method that brakes the robots wheels
    def brakeWheels(self):
        for i in self.powered_wheels:
            self.vehicle.setBrake(100, i)
    
    
    # a method that switches between types of control
    def setMoveMode(self, type: _CONTROLTYPES):
        self.mode = type
        if type == 'SpaceStart':
            self.space_start = False

    # a method that gives the robots powered wheels force if thay dont have it therefore starts movement
    def moveStart(self, engine_force: float):
        if not self.moving:
            for i in self.powered_wheels:
                self.vehicle.applyEngineForce(engine_force, i)
                self.vehicle.setBrake(0, i)
            self.moving = True

    # a method that ends the robots movement, optionally without braking the wheels
    def moveEnd(self, brake_wheels: bool = True, overwrite:bool = False):
        if self.moving or overwrite:
            for i in self.powered_wheels:
                self.vehicle.applyEngineForce(0, i)
                if brake_wheels:
                    self.vehicle.setBrake(100, i)
            self.moving = False
    
    # a method that gradually stops the robot
    def moveEndGradual(self, brake_wheels: bool = True, overwrite:bool = False):
        if self.moving or overwrite:
            for i in self.powered_wheels:
                self.vehicle.applyEngineForce(0, i)
                if brake_wheels:
                    self.vehicle.setBrake(self.default_force/100, i)
        if self.vehicle.getCurrentSpeedKmHour() < 1: 
            self.moving = False

   
    # a method that moves the robot forward and to the desired absolute target angle using PID, also returns true when 
    # it is finished
    def moveAndTurnPIDTimed(self, target_angle: float, time: float) -> bool:
        # add a new task if it doesn't exist yet
        if taskMgr.hasTaskNamed(self.move_task_name + 'PID') == False:
            def  movePID(task):
                # do this to unbrake wheels
                self.moveStart(0)
                # determine the current angle with regards to "forward" which is set up to mean (0, 1)
                unitY = Vec2.unitY()
                current_direction = self.vehicle.getForwardVector()
                current_direction = current_direction.xy
                current_direction = current_direction.normalized()
                current_angle = unitY.signedAngleDeg(current_direction)
                # print(current_angle)

                # ge tthe result from the pid and then give the wheels appropriate force
                wanted_engine_force = self.PID.calculate(target_angle, current_angle)
                if wanted_engine_force < 0:
                    self.vehicle.applyEngineForce(-wanted_engine_force, 0)
                    self.vehicle.applyEngineForce(-wanted_engine_force*0.1, 1)
                elif wanted_engine_force > 0:
                    self.vehicle.applyEngineForce(wanted_engine_force, 1)
                    self.vehicle.applyEngineForce(wanted_engine_force*0.1, 0)
                else:
                    self.vehicle.applyEngineForce(self.default_force/2, 1)
                    self.vehicle.applyEngineForce(self.default_force/2, 0)
                return task.cont
            
            # setup the timer and add the function to the task manager
            self.timer = self.clock.long_time + time
            taskMgr.add(movePID, self.move_task_name + 'PID')
        
        # remove the task once the wanted amount of time has passed
        if self.clock.long_time >= self.timer:
            self.moveEndGradual(overwrite=True, )
            taskMgr.remove(self.move_task_name + 'PID')
            return True
        else:
            return False


    
    # returns true when the robot is turned, otherwise returns false
    def turnFrontWheelRobot(self, relative_degrees: float) -> bool:
        # create a function and add it to task manager if it doesn't exist yet
        if taskMgr.hasTaskNamed(self.turn_task_name) == False:
            def turn(task):
                # turn one wheel forward, the other one back
                self.vehicle.applyEngineForce(self.default_force*2, 0)
                self.vehicle.applyEngineForce(-self.default_force*2, 1)
                return task.cont
            taskMgr.add(turn, self.turn_task_name)
            
            # calculate the target angle from the relative one
            unitY = Vec2.unitY()
            temp_vec = self.vehicle.getForwardVector()
            current_degs = unitY.signedAngleDeg(temp_vec.xy)
            abs_degrees = current_degs + (90 - relative_degrees)
            rads = radians(abs_degrees)
            # and assign it to self.target_direction
            self.target_direction = Vec2(cos(rads), sin(rads)).normalized()
            # print('new direction set' + str(self.target_direction.signedAngleDeg(unitY)))
        
        # once the robot is facing that direction (within a degree) remove the task and return true
        current_direction = self.vehicle.getForwardVector()
        current_direction = current_direction.xy
        current_direction = current_direction.normalized()
        if abs(current_direction.signedAngleDeg(self.target_direction)) < 1:
            taskMgr.remove(self.turn_task_name)
            self.moveEnd()
            return True
        else:
            return False
                    
    

    # this function returns True once the movement is finished, otherwise returns false
    def moveTimed(self, max_speed: float, time: float) -> bool:
            # create a function and add it to task manager if it doesn't exist yet
            if taskMgr.hasTaskNamed(self.move_task_name) == False:
                def moveTimedTemp(task):
                    # brake the wheels if the speed is above the target value
                    if abs(self.vehicle.getCurrentSpeedKmHour()) >= max_speed:
                        self.moveEnd(False)
                    # give the wheels force if the speed is below 80% of the thaget value
                    elif abs(self.vehicle.getCurrentSpeedKmHour()) < (max_speed*0.80):
                        self.moveStart(self.default_force)
                    return task.cont
                taskMgr.add(moveTimedTemp, self.move_task_name)
                # start a timer
                self.timer = self.clock.long_time + time
            # remove the function form the task manager once the timer is reached
            if self.clock.long_time >= self.timer:
                self.moveEndGradual(overwrite=True)
                taskMgr.remove(self.move_task_name)
                return True
            else:
                return False

    # program the demo, in this one the bot first turns 90 degrees, then moves forward and then moves and turns using PID controller 
    def moveDemoFrontWheelRobot(self):
        match self.move_demo_stage:
            case 0:
                if self.moveAndTurnPIDTimed(20, 3): 
                    self.move_demo_stage = 1
                    print('now in demo stage ' + str(self.move_demo_stage))
            case 1: 
                if self.moveAndTurnPIDTimed(-20, 3):
                    self.move_demo_stage = 0
                    print('now in demo stage ' + str(self.move_demo_stage))


            


# create a pid obcect that can be refernced to calculate the wheel force
class PIDObject():
    def __init__(self, P, I, D):
        # setup constant and additional variables
        self.P = P
        self.I = I
        self.D = D
        self.last_value = 0
        self.error_sum = 0

    
    # calculate pid
    def calculate(self, current_terget: float, current_value: float) -> float:
        error = current_terget - current_value
        self.error_sum = self.error_sum + error
        dt = globalClock.getDt()
        derivative = (error - self.last_value)/dt
        res = error * self.P + self.error_sum * self.I + derivative * self.D
        self.last_value = error
        return res       
                
            

            
        

       


# a function that creates a robot with two nodes and a specified configuration of wheels
def createTwoStoryRobot(world: BulletWorld, base_size: Vec3, upper_size: Vec3, dist: float, wheel_pos: _WHEELPOSTYPES, name: str, 
                        position=Vec3(0, 0, 3), base_mass_mult: float = 1, PID: bool = True) -> Robot:
    
    parts = []
    powered_wheels = []

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

    # connect the nodes
    cons = helper.connectNodesVertical(world, base, node, dist)


    # add the part and its connector to a list
    parts.append([node, cons])

    # create pid (this could probably be calculated from the mass and dimensions of the robot but I didn't manage to do it)
    if PID:
        pid = PIDObject(9, 0.05, 0)
    else: 
        pid = PIDObject(0, 0, 0)

    # add wheels based on chosen configuration (note: 'Front' is the main configuration, the
    # others were added to show that different ones were possible)
    match wheel_pos:
        case 'Front':
            powered_wheels = [0, 1]
            # create the two big front wheels
            for i in range(2):
                wheel = vehicle.createWheel()


                wheel.setChassisConnectionPointCs(Vec3(base_size.getX()*pow(-1, i), -base_size.getY(), 0))
                wheel.setWheelRadius(base_size.getZ()*1.3)
                wheel.setWheelDirectionCs(Vec3(0, 0, -1))
                wheel.setWheelAxleCs(Vec3(-1, 0, 0))
                wheel.setMaxSuspensionTravelCm(base_size.getZ()*60) #important

                wheel.setSuspensionStiffness(base_size.getZ()*20) # important
                wheel.setWheelsDampingRelaxation(2.3)
                wheel.setWheelsDampingCompression(4.4)
                wheel.setFrictionSlip(20)
                wheel.setRollInfluence(0.1)
                wheel.setFrontWheel(True)



            # create the small back wheel
            wheel = vehicle.createWheel()


            wheel.setChassisConnectionPointCs(Vec3(0 , base_size.getY() + 0.1, 0))
            wheel.setWheelRadius(base_size.getZ())
            wheel.setWheelDirectionCs(Vec3(0, 0, -1))
            wheel.setWheelAxleCs(Vec3(1, 0, 0))
            wheel.setMaxSuspensionTravelCm(base_size.getZ()*40) #important
            wheel.setSuspensionStiffness(80) # important
            wheel.setWheelsDampingRelaxation(2.3)
            wheel.setWheelsDampingCompression(4.4)
            wheel.setFrictionSlip(100)
            wheel.setRollInfluence(0)

        
        # create wheels on the corners that are perpendicular to the diagonals of the robots body 
        # (currently this robot can only spin, but such a configuration is somewhat common so I added it for posterity, 
        # usually they work by two slip wheels spinning in opposite directions to move along the line that is their bisector)
        case 'Corners':
            powered_wheels = [0, 1, 2, 3]
            corners = [Vec3(base_size.getX(), base_size.getY(), 0), Vec3(-base_size.getX(), base_size.getY(), 0), 
                    Vec3(base_size.getX(), -base_size.getY(), 0), Vec3(-base_size.getX(), -base_size.getY(), 0)]
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
                wheel.setRollInfluence(0)
                wheel.setFrontWheel(False)

        # this configuration has a single wheel in the middle
        case 'OneWheel':
            powered_wheels = [0]
            wheel = vehicle.createWheel()
            wheel.setChassisConnectionPointCs(Vec3(0, 0, 0))
            wheel.setWheelRadius(base_size.getZ()*1.1)
            wheel.setWheelDirectionCs(Vec3(0, 0, -1))
            wheel.setWheelAxleCs(Vec3(-1, 0, 0))            
            wheel.setMaxSuspensionTravelCm(base_size.getZ()*60) #important

            wheel.setSuspensionStiffness(base_size.getZ()*20) # important
            wheel.setWheelsDampingRelaxation(2.3)
            wheel.setWheelsDampingCompression(4.4)
            wheel.setFrictionSlip(20)
            wheel.setRollInfluence(0.1)
            wheel.setFrontWheel(False)


    # create the robot
    robot = Robot(vehicle, parts, powered_wheels, name, pid)
    # brake the wheels and return the robot
    robot.moveEnd()
    return robot

