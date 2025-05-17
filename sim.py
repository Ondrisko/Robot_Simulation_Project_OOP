import direct.showbase.ShowBase as ShowBase
from panda3d.core import Vec3
import RoboLib as rl
import helper


def main() -> None:

    # create the world using a function from the helper file
    world = helper.worldSetup(True)


    # create a robot with given dimensions
    robot = rl.createTwoStoryRobot(world, Vec3(3, 3.5, 2), Vec3(0.5, 0.5, 0.5), 6, 'Front', 'Robot1', base_mass_mult=1)
    robot.setMoveMode('Demo')


    # it is possible to create a different robot, however these specific dimensions cause erratic behaviour
    robot2 = rl.createTwoStoryRobot(world, Vec3(3, 3, 2), Vec3(2, 2, 2), 4.1, 'Corners', 'Robot2', position=Vec3(10, 10, 3))
    robot2.setMoveMode("Demo")

    # run the panda3d scene
    children = render.getChildren()
    print(children)
    print(taskMgr)
    base.run()



    

if __name__ == "__main__":
    main()