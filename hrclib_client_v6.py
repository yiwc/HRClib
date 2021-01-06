# Python3 Interfaces for ARC actions libraries
# It connects to py2 servers, with py3 clients
# Email: yiwen.chen@u.nus.edu
import rospy
import actionlib
import movo_arc_lib.msg as arcmsg
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
import threading
import numpy as np
import math

# human robot collaboration library
# HRClib


# Icarus architecture
def motion(cmd, args, goal, eval, type):
    if not isinstance(args, tuple):
        args = (args,)
    while True:
        cmd(*args)
        if type == 1:
            dist = sum([(v - goal[i]) ** 2 for i, v in enumerate(eval())]) ** 0.5
            if dist < 0.01:
                break
        elif type == 2:
            if eval() == goal:
                break
        elif type == 3:
            if ((eval() - goal) ** 2) ** 0.5 < 0.001:
                break
        elif type == 4:
            res = eval()
            val = [res == goal[i] for i in range(len(goal))]
            if sum(val) > 0:
                break
class GlobalVariables(object):
    def __init__(self):
        # self.default_pose_tucked = [-1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
        #                             1.6, 1.5, -0.4, 2.7, 0.0, -0.5, 1.7,
        #                             0.04, 0, 0]

        self.default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
                                    1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
                                    0.14, 0, -0.6]
        self.default_pose_wings =[-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0,
                                  2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0,
                                  0.42, 0, 0]  # Preparing Grasping
        self.default_pose_hug= [-0.23, -0.71, -1.02, -1.0, 0.9, 1.89, -2.41,
                                0.44, 0.71, 1.12, 1.21, -1.02, -1.84,2.61, 0.20, 0, 0] # preparing for hold two hands
        self.default_pose_pickready = [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,#-1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
                                       -0.642, 0.936, -0.767, 1.46,0.928, -1.68, -2.39,
                                      0.46, 0, 0]
        self.default_pose_pickready_lookdown = [0.642, -0.936, 0.767, -1.46, -0.928, 1.68, 2.39,
                                       # -1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7,
                                       -0.642, 0.936, -0.767, 1.46, 0.928, -1.68, -2.39,
                                       0.46, 0, -0.6]
        self.default_pose_horizon_rot_before_insert_ready = \
            [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,-1.109,1.122,1.343,1.6119,-0.560,-1.846,2.996,
                                              0.46, 0, -0.6]
        self.default_pose_bolt_pick_ready_pose = [0.477, -0.925, -0.55, -1.550, 1.166, 1.749, 0.704,
           -0.642, 0.936, -0.767, 1.46,0.928, -1.68, -2.39,
           0.46, 0, -0.6]
        self.default_pose_ready_using_screw_driver = [0.642, -0.936, 0.767, -1.46,-0.928, 1.68, 2.39,
                   -0.477, +0.925, +0.55, +1.550,-1.166,-1.749,-0.704,
                   0.46, 0, -0.6]
class myclient(object):
    def __init__(self,name,action,fbmsg):
        self._client=actionlib.SimpleActionClient(name,action)
        print("Connecting...",name)
        self._client.wait_for_server()
        print("Connected",name)
        rospy.Subscriber("/" + name + "/feedback", fbmsg,
                         self._subscribe_actionfb_callback)
        self.fbdata=None

    def __getattr__(self, item):
        return getattr(self._client, item)
    def result_published(self):
        res = self._client.get_result()
        # self._client.
        # if(res.success)
        if (type(res) == type(None)):
            return False
        else:
            return True

    def _subscribe_actionfb_callback(self,data):
        self.fbdata=data.feedback

class odyssey_Interface():
    # simple interface name with L0...
    # ICURAS interface name with i_L0...
    def __init__(self):
        self.gval=GlobalVariables()
        rospy.init_node("odyssey_Interface_py3_node")

        self.client_L0_upper_jp_move_safe=myclient(name="L0_upper_jp_move_safe",
                                                   action=arcmsg.upper_jp_movo_safeAction,
                                                   fbmsg=arcmsg.upper_jp_movo_safeFeedback)

        self.client_L0_dual_jp_move_safe_relate=myclient(name="L0_dual_jp_move_safe_relate",
                                                   action=arcmsg.dual_jp_movo_safe_relateAction,
                                                   fbmsg=arcmsg.dual_jp_movo_safe_relateFeedback)

        self.client_L0_dual_task_move_safe_relate=myclient(name="L0_dual_task_move_safe_relate",
                                                           action=arcmsg.dual_task_move_safe_relateAction,
                                                           fbmsg=arcmsg.dual_task_move_safe_relateFeedback)

        self.client_L0_dual_set_gripper=myclient(name="L0_dual_set_gripper",
                                                 action=arcmsg.dual_set_gripperAction,
                                                 fbmsg=arcmsg.dual_set_gripperFeedback)


        self.client_L0_single_task_move_safe=myclient(name="L0_single_task_move_safe",
                                                 action=arcmsg.single_task_move_safeAction,
                                                 fbmsg=arcmsg.single_task_move_safeFeedback)


        rospy.Subscriber("/joint_states", JointState,self._subscribe_jointstates_callback)

        self.jp_updated=False
        r=rospy.Rate(10)
        while(not (self.jp_updated and True)):
            r.sleep()



    def _subscribe_jointstates_callback(self,data):
        self.linear_js_pos = data.position[0:1]
        self.linear_js_vel = data.velocity[0:1]

        self.head_js_pos = data.position[1:3] #
        self.head_js_vel = data.velocity[1:3] #

        self.right_arm_js_pos = data.position[4:11]
        self.right_arm_js_vel = data.velocity[4:11]

        self.left_arm_js_pos = data.position[11:18]
        self.left_arm_js_vel = data.velocity[11:18]

        self.jp=self.right_arm_js_pos+self.left_arm_js_pos+self.linear_js_pos+self.head_js_pos

        self.jp_updated = True
    def E0_getjp(self):
        return self.jp
    def E0_getjp_arm(self,arm):

        if(arm=="left" or arm=='l'):
            pos=self.left_arm_js_pos
            print(pos)
            return pos
        if(arm=="right" or arm=='r'):
            return self.right_arm_js_pos
        raise NotImplementedError("ARM not defined")
        # return self.jp


    def tool_dist(self,l1,l2):
        dist = sum([(v - l1[i]) ** 2 for i, v in enumerate(l2)]) ** 0.5
        return dist


    def set_grippers(self,**kwargs):
        return self._L0_dual_set_gripper(**kwargs)
    def arm_cart_move(self,**kwargs):
        return self._L0_single_task_move_safe(**kwargs)
    def arms_cart_move(self,**kwargs):
        return self._L0_single_task_move_safe(**kwargs)
    def single_move_relate(self,**kwargs):
        return self._L1_single_task_move_safe_relate(**kwargs)

    class Decorators(object):
        @classmethod
        def L_Action(self,fun):
            def do_action(self,*args,**kwargs):
                print("Call->",str(fun.__name__)," : ",args,kwargs)
                fun(self,*args,**kwargs)
            return do_action

    @Decorators.L_Action
    def _L1_single_task_move_safe_relate(self,arm,move,maxforce,time,wait=True,hard=False):
        assert arm in ["left","right"]
        if hard:
            maxforce=[1000 for i in range(6)]


        rmove=[0,0,0]
        lmove=[0,0,0]
        time=time
        rmaxforce=lmaxforce=[1000 for i in range(6)]

        if arm =="right":
            rmove=move
            rmaxforce=maxforce
        elif arm =="left":
            lmove=move
            lmaxforce=maxforce


        # def _L0_dual_task_move_safe_relate(self, rmove, lmove, time, rmaxforce, lmaxforce, wait=True, h

        self._L0_dual_task_move_safe_relate(rmove,lmove,time,rmaxforce,lmaxforce,wait,hard)

    @Decorators.L_Action
    def _L0_dual_set_gripper(self,value,wait=True):
        goal=arcmsg.dual_set_gripperGoal
        goal.value=value
        self.done_thr_set_grippers=False
        self.client_L0_dual_set_gripper.send_goal(goal)
        if wait:
            self.client_L0_dual_set_gripper.wait_for_result()
        self.done_thr_set_grippers=True
    @Decorators.L_Action
    def _L0_dual_jp_move_safe_relate(self, jp_r, jp_l, lmaxforce, rmaxforce, duration, wait=True,hard=False):
        if hard:
            f=1000
            lmaxforce=rmaxforce=[f for i in range(6)]
        self.done_thr_jprot_re = False
        mygoal = arcmsg.dual_jp_movo_safe_relateGoal()
        mygoal.jp_left_relate = jp_l
        mygoal.jp_right_relate = jp_r
        mygoal.l_max_force = lmaxforce
        mygoal.r_max_force = rmaxforce
        mygoal.duration = duration
        # send a goal
        if (wait):
            self.client_L0_dual_jp_move_safe_relate.send_goal_and_wait(mygoal)
        else:

            self.client_L0_dual_jp_move_safe_relate.send_goal(mygoal)
        pass
        self.done_thr_jprot_re = True

    # @Decorators.L_Action
    def _L0_single_task_move_safe(self,arm,pos,orn,maxforce,wait=True,hard=False):
        if hard:
            f=1000
            maxforce=[f for i in range(6)]
        assert arm in ["left","right"]
        self.done_thr_single_task=False
        mygoal = arcmsg.single_task_move_safeGoal()
        arm = 0 if arm=="left" else 1
        mygoal.pos = pos
        mygoal.orn = orn
        mygoal.arm = arm
        mygoal.max_force = maxforce
        # mygoal.duration = duration
        # send a goal
        if(wait):
            self.client_L0_single_task_move_safe.send_goal_and_wait(mygoal)
        else:
            self.client_L0_single_task_move_safe.send_goal(mygoal)

        self.done_thr_single_task=True

    @Decorators.L_Action
    def _L0_dual_task_move_safe_relate(self,rmove,lmove,time,rmaxforce,lmaxforce,wait=True,hard=False):

        if hard:
            f=1000
            lmaxforce=rmaxforce=[f for i in range(6)]

        self.done_thr_taskmov_re=False
        mygoal = arcmsg.dual_task_move_safe_relateGoal()
        mygoal.pos_r=rmove
        mygoal.pos_l=lmove
        mygoal.time=time
        mygoal.r_max_force=rmaxforce
        mygoal.l_max_force=lmaxforce

        # send a goal
        if wait:
            self.client_L0_dual_task_move_safe_relate.send_goal_and_wait(mygoal)
        else:
            self.client_L0_dual_task_move_safe_relate.send_goal(mygoal)
            # self.client_L0_dual_task_move_safe_relate.wait_for_result()

        self.done_thr_taskmov_re=True

    @Decorators.L_Action
    def _L0_upper_jp_move_safe(self,jpl,jpr,jph,jplinear,duration,lforce,rforce,wait=True,hard=False):

        if hard:
            f=1000
            lforce=rforce=[f for i in range(6)]

        mygoal = arcmsg.upper_jp_movo_safeGoal()
        mygoal.jp_left = jpl
        mygoal.jp_right = jpr
        mygoal.jp_head = jph
        mygoal.jp_linear = jplinear
        mygoal.duration = duration
        mygoal.l_max_force = lforce
        mygoal.r_max_force = rforce
        self.client_L0_upper_jp_move_safe.send_goal(mygoal)
        if wait:
            self.client_L0_upper_jp_move_safe.send_goal_and_wait(mygoal)
        else:
            self.client_L0_upper_jp_move_safe.send_goal(mygoal)

    # deprecated
    def eval_L0_upper_jp_move_safe(self):
        return self.E0_getjp()
    def _L0_dual_jp_move_safe_relate_decodeargs(self,args):
        self._L0_dual_jp_move_safe_relate(jp_r=args[:7],
                                          jp_l=args[7:14],
                                          rmaxforce=args[14:20],
                                          lmaxforce=args[20:26],
                                          duration=args[26])
    def L0_upper_jp_move_safe(self,args):

        #filter_inputs
        # TODO
            # same input or not
        lastargs=self.client_L0_upper_jp_move_safe_last_args
        newargs=args
        goal_state=self.client_L0_upper_jp_move_safe.get_state()
        if(goal_state in [GoalStatus.PREEMPTED,GoalStatus.SUCCEEDED]):
            #force reached maximum
            pass
            # TODO
            # currently, go on sending goal
        else:
            if(not type(lastargs)==type(None)):
                dist=self.tool_dist(newargs,lastargs)
                if(dist<0.01):
                    return
        self.client_L0_upper_jp_move_safe_last_args=newargs[:]
        #decode args
        mygoal = arcmsg.upper_jp_movo_safeGoal()
        mygoal.jp_left = args[7:14]
        mygoal.jp_right = args[0:7]
        mygoal.jp_head = args[15:17]
        mygoal.jp_linear = args[14:15][0]
        mygoal.duration = 300
        mygoal.l_max_force = args[17:23]
        mygoal.r_max_force = args[23:29]

        # send a goal
        self.client_L0_upper_jp_move_safe.send_goal(mygoal)

# Stable Tests
def routine_test_all():
    # A script to test L0_dual_task_move_safe_relate with ICARUS
    force_left = [10 for i in range(6)]
    force_right = [10 for i in range(6)]
    args_input_1 = arc.gval.default_pose_pickready[:] + force_left + force_right
    args_input_2 = arc.gval.default_pose_horizon_rot_before_insert_ready[:] + force_left + force_right

    # gripper test
    arc._L0_dual_set_gripper(1)
    arc._L0_dual_set_gripper(0)
    arc._L0_dual_set_gripper(1)
    arc._L0_dual_set_gripper(0)

    # jo relate move
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 0], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, 1.8], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, -1.8], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, 0], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)
    arc._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 1.8], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0, 0, 0, 0, 0, -1.8], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)

    # task move test
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, 0.1, 0], rmaxforce=[10 for i in range(6)],
        lmove=[0, -0.1, 0], lmaxforce=[10 for i in range(6)],
        time=3)
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, -0.1, 0], rmaxforce=[10 for i in range(6)],
        lmove=[0, +0.1, 0], lmaxforce=[10 for i in range(6)],
        time=3)

    print("motion finished", "_L0_dual_jp_move_safe_relate")
    rospy.spin()

def routine_test():
    # arc.client_L0_single_task_move_safe()
    force=50
    for i in range(3):
        print(i)
        arc._L0_single_task_move_safe("right",[0.74,-0.45,1.17],
                                      [0., math.pi / 2, -math.pi / 2],
                                      [force for i in range(6)])
        arc._L0_single_task_move_safe("right",[0.74,-0.25,1.17],
                                      [0., math.pi / 2, -math.pi / 2],
                                      [force for i in range(6)])
if __name__=="__main__":
    print("Start")

    arc=odyssey_Interface()
    routine_test()
