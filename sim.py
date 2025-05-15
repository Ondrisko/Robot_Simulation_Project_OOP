import direct.showbase.ShowBase as ShowBase
from panda3d.core import Vec3
import RoboLib as rl
import helper


def main() -> None:

    world = helper.worldSetup(True)

    robot = rl.createTwoStoryRobot(world, Vec3(3, 3, 1), Vec3(1, 1, 0.5), 4, 'Front')

    robot2 = rl.createTwoStoryRobot(world, Vec3(3, 3, 2), Vec3(2, 2, 2), 4.1, 'Front', Vec3(10, 10, 10))


    base.run()


    

if __name__ == "__main__":
    main()