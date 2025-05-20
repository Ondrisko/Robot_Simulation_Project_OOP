import RoboLib as rl
import helper
from panda3d.core import Vec3


def main() -> None:

    world = helper.worldSetup(True)
    base.cam.setPos(Vec3(0, 300, 150))

    base_size = Vec3(3, 3, 1)
    part_size = Vec3(1, 1, 1)
    
    # this robot's PID is not set up (i.e. all its parts are 0) and any movement to the sides is caused by bugs either in bullet
    # or the specifin implementation I chose
    robot1 = rl.createTwoStoryRobot(world, base_size, part_size, 3, "Front", "NoPID", Vec3(-100, 0, 3), PID=False, base_mass_mult=3)
    robot1.setMoveMode("Demo")

    print(robot1.PID)

    # this robot's PID is default (i.e. 9, 0.01, 0) it moves erratically and is prone to mistakes
    robot2 = rl.createTwoStoryRobot(world, base_size, part_size, 3, "Front", "DefaultPID", Vec3(100, 0, 3), PID=True, base_mass_mult=3)
    robot2.setMoveMode("Demo")

    # this robot's PID is somewhat sensible and so it moves side to side (given it doesn't run into the other robots)
    robot3 = rl.createTwoStoryRobot(world, base_size, part_size, 3, "Front", "AdjustedPID", base_mass_mult=3)
    robot3.PID = rl.PIDObject(4, 0, 0.01)
    robot3.setMoveMode("Demo")

    base.run()



if __name__ == "__main__":
    main()