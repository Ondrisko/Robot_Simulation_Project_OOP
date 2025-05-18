from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3, Point3, NodePath
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletCapsuleShape, BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletWorld
from panda3d.bullet import BulletSphereShape, BulletConeTwistConstraint, BulletGenericConstraint
from panda3d.core import TransformState

# a function to create a plane and give it a texture
def createPlane(world) -> None:
    shape = BulletPlaneShape(Vec3(0, 0, 1), 1)
    node = BulletRigidBodyNode('Ground')
    node.addShape(shape)
    np = render.attachNewNode(node)
    np.setPos(0, 0, -1)
    world.attachRigidBody(node)
    model = loader.loadModel('models/box.egg')
    texture = loader.loadTexture('Grid.jpg')
    model.setPos(-750, -750, 0)
    model.setScale(1500, 1500, 1)
    model.reparentTo(np)
    model.setTexture(texture, 1)


# connect two nodes vertically without movement at the constraint joint
def connectNodesVertical(world: BulletWorld, node1: BulletRigidBodyNode, node2: 
                 BulletRigidBodyNode, distance: float) -> BulletGenericConstraint:
    
    # create the connection and diasble movement at the joint
    cons = BulletGenericConstraint(node1, node2, TransformState.makePos(Vec3(0, 0, distance/2)), 
                                   TransformState.makePos(Vec3(0, 0, -distance/2)), False)
    cons.setAngularLimit(1, 0, 0)
    cons.setAngularLimit(0, 0, 0)
    cons.setAngularLimit(2, 0, 0)
    world.attachConstraint(cons)
    return cons

# create a function that follows a given NodePath
def cameraFollow(np: NodePath):
    def cam(task):
        pos = np.getPos()
        pos = pos.__add__(Vec3(0, 70, 20))
        base.cam.setPos(pos)
        base.cam.lookAt(np.getPos())
        return task.cont
    taskMgr.add(cam, "cam")

# do all the miscellaneous world setup
def worldSetup(debug = False) -> BulletWorld:
    base = ShowBase()
    
    # set camera position
    base.cam.setPos(0, 70, 20)
    base.cam.lookAt(0, 0, 5)

   
    


    # create world for bullet
    world = BulletWorld()
    world.setGravity(Vec3(0, 0, -9.81))
    
    # show debug meshes if they are enabled
    if debug:
        dn = BulletDebugNode('Debug')
        dn.showWireframe(True)
        dnp = render.attachNewNode(dn)
        dnp.show()
        world.setDebugNode(dnp.node())


    createPlane(world)

    # create a task which calculates bullet physics for the panda task manager and add it
    def update(task):
        dt = globalClock.getDt()
        world.doPhysics(dt)
        return task.cont
    taskMgr.add(update, 'update')

    return world