import atwork_commander_msgs.msg
import arena_info
import service_area
from franka_modules.panda_move_modules import makeAllObjStack, panda_init
import rospy
import utils_move
import arena_info 


        
def generate_example_task(obj_type):
    obj = atwork_commander_msgs.msg.Object()
    obj.object = obj_type
    task = ["WS07", "PP01", obj.object]
#     task = ["WS01", "WS01", obj.object]
    return task

def test_all_objects(): # gray and black are two objects! 
    robot, scene, moveGroupArm, moveGroupHand = panda_init()
    makeAllObjStack(scene)

    strings = rospy.get_param('/objects_manipulation/').keys()
    IDs = []
    for i in range(len(strings)):
        IDs += [rospy.get_param('/objects_manipulation/' + strings[i] + '/ID')]

    for i, obj_id in enumerate(IDs):

        workstation = service_area.Service_area("WS02", n)
        task = generate_example_task(obj_id)
        arena_info.process_objects(task, workstation)

        name = utils_move.object_id_to_string(task[2].object)
        utils_move.remove_obj(name + '#' + str(i))

if __name__ == "__main__":
    node = rospy.init_node('arena_test', anonymous=True)

    #test_all_objects()
    robot, scene, moveGroupArm, moveGroupHand = panda_init()
    makeAllObjStack(scene, moveGroupArm)

    tasks = [generate_example_task(atwork_commander_msgs.msg.Object.MOTOR)] # task with one object
    #tasks.append(generate_example_task(atwork_commander_msgs.msg.Object.))

    #for t in task:
    #task[0][2] = task[0][2].object
    #task[1][2] = task[1][2].object

    #Workstations = ["WS01", "WS02","WS03", "WS04", "WS05", "WS06", "WS07", "WS08", "WS09", "SH01top", "SH01bottom", "PP01"]
    #for Workstation in Workstations:
    #    arena_info.move_client(arena_info.create_moveStamped_task(Workstation), node)


<<<<<<< HEAD
    workstation = service_area.Service_area("WS02", node)
    arena_info.process_objects(tasks, workstation)
    utils_move.remove_obj("S40_40_G" + '#' + str(0))
=======
    utils_move.home_gripper()
    arena_info.process_objects(tasks, node, scene)
    #utils_move.remove_obj("S40_40_G" + '#' + str(0))
>>>>>>> robot_urdf
