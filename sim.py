import direct.showbase.ShowBase as ShowBase
from panda3d.core import Vec3
import RoboLib as rl
import helper


def main() -> None:

    # create the world using a function from the helper file
    world = helper.worldSetup(True)


    # create a robot with given dimensions
    robot = rl.createTwoStoryRobot(world, Vec3(3, 3, 1), Vec3(1, 1, 0.5), 4, 'Front')


    # it is possible to create a different robot, however these specific dimensions cause erratic behaviour
    # robot2 = rl.createTwoStoryRobot(world, Vec3(3, 3, 2), Vec3(2, 2, 2), 4.1, 'Front', Vec3(10, 10, 10))

    # run the panda3d scene
    base.run()


    

if __name__ == "__main__":
    main()