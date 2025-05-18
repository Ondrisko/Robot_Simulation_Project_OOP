import direct.showbase.ShowBase as ShowBase
from panda3d.core import Vec3
import RoboLib as rl
import helper


def main() -> None:

    # create the world using a function from the helper file
    world = helper.worldSetup(True)

    
    # create a robot with given dimensions
    robot = rl.createTwoStoryRobot(world, Vec3(3, 3.5, 2), Vec3(0.5, 0.5, 0.5), 6, 'Front', 'Robot1', base_mass_mult=3)
    robot.setMoveMode('SpaceStart')
    
    # set the camera to follow the robot
    helper.cameraFollow(robot.chassis_np)


    # it is possible to create a different robot, however these specific dimensions cause erratic behaviour
    # robot2 = rl.createTwoStoryRobot(world, Vec3(3, 3, 2), Vec3(2, 2, 2), 4.1, 'Corners', 'Robot2', position=Vec3(10, 10, 3))
    # robot2.setMoveMode("Forward")
    
    # this loop creates a squad of small robots that go forward, added to show that they can be added via a loop 
    # which was ultimately my goal
    '''
    for i in range(20):
        name = str('robot' + str(i))
        robot = []
        temp_robot = rl.createTwoStoryRobot(world, Vec3(1, 1, 0.4), Vec3(0.5, 0.5, 0.5), 
                                            1, 'OneWheel', name, position=Vec3(i*3 - 30, 0, 2), base_mass_mult=5)
        temp_robot.setMoveMode('Forward')
        robot.append(temp_robot)
    '''

    # run the panda3d scene
    children = render.getChildren()
    print(children)
    print(taskMgr)
    base.run()

    

if __name__ == "__main__":
    main()