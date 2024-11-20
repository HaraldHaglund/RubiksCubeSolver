# TODO rename all private variables to start with '_'
# TODO: neither visualization works without printing from manager, 
#       make this make sense dude (but it does work with that, so this aint prio)
# TODO: just read the q and update everything every timestep, don't deepcopy,
# TODO: rewrite all getSomething functions to updateSomething functions,
#       and then make the getSomething function just be return self.something.copy()
# --> just create a RobotManager.step() function, update everything there
# don't do forwardKinematics 2 extra times for no good reason. make that the libraries
# responsibility, not the users
import pinocchio as pin
import numpy as np
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
from ur_simple_control.util.grippers.robotiq.robotiq_gripper import RobotiqGripper
from ur_simple_control.util.grippers.on_robot.twofg import TWOFG
import copy
import signal
from ur_simple_control.util.get_model import get_model
from collections import deque
from ur_simple_control.visualize.visualize import plotFromDict, realTimePlotter, manipulatorVisualizer
from multiprocessing import Process, Queue

"""
general notes
---------------
The first design principle of this library is to minimize the time needed
to go from a control algorithm on paper to the same control algorithm 
running on the real robot. The second design principle is to have
the code as simple as possible. In particular, this pertains to avoiding
overly complex abstractions and having the code as concise as possible.
The user is expected to read and understand the entire codebase because
changes will have to accomodate it to their specific project.
Target users are control engineers.
The final design choices are made to accommodate these sometimes opposing goals
to the best of the author's ability.

This file contains a robot manager and a control loop manager.
The point of these managers is to handle:
    - boiler plate code around the control loop which is always the same
    - have all the various parameters neatly organized and in one central location
    - hide the annoying if-elses of different APIs required 
      for the real robot and various simulations with single commands
      that just do exactly what you want them to do


current state
-------------
Everything is UR specific or supports pinocchio only simulation,
and only velocity-controlled robot functions exist.

long term vision
-------------------
Cut out the robot-specific parts out of the manager classes,
and create child classes for particular robots.
There is an interface to a physics simulator.
Functions for torque controlled robots exist.
"""

class ControlLoopManager:
    """
    ControlLoopManager
    -------------------
    Slightly fancier programming (passing a function as argument and using functools.partial)
    to get a wrapper around the control loop.
    In other words, it's the book-keeping around the actual control loop.
    It's a class because it keeps non-directly-control-loop-related parameters
    like max_iterations, what data to save etc.
    NOTE: you give this the ready-made control loop.
    if it has arguments, bake them in with functools.partial.
    Handles short-term data saving and logging.
    Details on this are given below.

    Short term data saving:
            - it's a dictionaries of deques (initialized here), because deque is the most convenient class 
              for removing the first element and appending a last (it is just a linked list under the hood of course).
            - it's a dictionary for modularity's sake, because this way you can save whatever you want
            - and it will just work based on dictionary keys.
            - it is the user's resposibility to make sure they're providing correct data.
            - --> TODO but make an assert for the keys at least
            - in the c++ imlementation, make the user write their own struct or something.
            - since this is python, you need to give me initial values to infer types.
            - you need to provide initial values to populate the deque to start anyway.

    Logging data (for analysis and plotting):
            - it can only be handled here because the control loop itself only cares about the present/
              a small time-window around it.
            - saves it all in a dictionary of ndarrays (initialized here), returns that after a run
              TODO: it's provided by the user now, make it actually initialize here!!!
            - you need to specify which keys you're using to do the initialization 
            - later, the controlLoop needs to return what's to be save in a small temporary dict.
            - NOTE: this is of course a somewhat redundant solution, but it's the simplest
              and most flexible way of doing it. 
              it probably will be done some other way down the line, but it works and is not
              a priority right now

    Other info:
    - relies on RobotManager to handle all the magic numbers 
      that are not truly only control loop related

    """

    def __init__(self, robot_manager, controlLoop, args, save_past_item, log_item):
        signal.signal(signal.SIGINT, self.stopHandler)
        self.max_iterations = args.max_iterations
        self.robot_manager = robot_manager
        self.controlLoop = controlLoop
        self.final_iteration = -1 # because we didn't even start yet
        self.args = args
        self.past_data = {}
        # save_past_dict has to have the key and 1 example of what you're saving
        # so that it's type can be inferred (but we're in python so types don't really work).
        # the good thing is this way you also immediatelly put in the initial values
        for key in save_past_item:
            self.past_data[key] = deque()
            # immediatelly populate every deque with initial values
            for i in range(self.args.past_window_size):
                # deepcopy just in case, better safe than sorry plus it's during initialization,
                # not real time
                self.past_data[key].append(copy.deepcopy(save_past_item[key]))

        # similar story for log_dict as for past_data,
        # except this is not used in the control loop,
        # we don't predeclare sizes, but instead
        # just shove items into linked lists (python lists) in dictionaries (hash-maps)
        self.log_dict = {}
        for key in log_item:
            self.log_dict[key] = []

        if self.args.real_time_plotting:
            self.plotter_queue = Queue()
            if self.args.debug_prints:
                print("CONTROL_LOOP_MANAGER", self.controlLoop, ": i created queue for real time plotting:", self.plotter_queue)
                print("CONTROL_LOOP_MANAGER: i am creating and starting the real-time-plotter  process")
            self.real_time_plotter_process = Process(target=realTimePlotter, 
                                                     args=(self.args, self.plotter_queue, ))
            # give real-time plotter some time to set itself up
            self.real_time_plotter_process.start()
            if self.args.debug_prints:
                print("CONTROL_LOOP_MANAGER: real_time_plotter_process started")
            # wait for feedback that the thing has started
            self.plotter_queue.get()
            self.plotter_queue.put(log_item)
            if self.args.debug_prints:
                print("CONTROL_LOOP_MANAGER: i managed to put initializing log_item to queue")


    def run(self):
        """
        run
        ---
        do timing to run at 500Hz.
        also handle the number of iterations.
        it's the controlLoop's responsibility to break if it achieved it's goals.
        this is done via the breakFlag
        """
        self.final_iteration = 0
        for i in range(self.max_iterations):
            start = time.time()
            # NOTE: all required pre-computations are handled here
            self.robot_manager._step()
            # TODO make the arguments to controlLoop kwargs or whatever
            # so that you don't have to declare them on client side if you're not using them
            breakFlag, latest_to_save_dict, log_item = self.controlLoop(i, self.past_data)
            self.final_iteration = i

            # update past rolling window
            # TODO: write an assert assuring the keys are what's been promised
            # (ideally this is done only once, not every time, so think whether/how that can be avoided)
            for key in latest_to_save_dict:
                # remove oldest entry
                self.past_data[key].popleft()
                # add new entry
                self.past_data[key].append(latest_to_save_dict[key])
            
            # log the data
            # check that you can
            # TODO only need to check this once, pls enforce better
            #if len(self.log_dict) > 0:
            for key in log_item:
                    #if key not in self.log_dict.keys():
                    #    raise KeyError("you need to provide log items you promised!")
                    #    break
                        #self.robot_manager.stopHandler(None, None)
                self.log_dict[key].append(log_item[key])
            
            if i % 20 == 0:
                # don't send what wasn't ready
                if self.args.visualize_manipulator:
                    if self.robot_manager.manipulator_visualizer_queue.qsize() < 5:
                        self.robot_manager.manipulator_visualizer_queue.put_nowait(self.robot_manager.q)
#                    if self.args.debug_prints:
#                        print("manipulator_visualizer_queue size status:", self.robot_manager.manipulator_visualizer_queue.qsize())
                if self.args.real_time_plotting:
                    # don't put new stuff in if it didn't handle the previous stuff.
                    # it's a plotter, who cares if it's late. 
                    # the number 5 is arbitrary
                    if self.plotter_queue.qsize() < 5:
                        self.plotter_queue.put_nowait(log_item)
                    #print("plotter_queue size status:", self.plotter_queue.qsize())

            # break if done
            if breakFlag:
                break

            # sleep for the rest of the frequency cycle
            end = time.time()
            diff = end - start
            if self.robot_manager.dt < diff:
                if self.args.debug_prints:
                    print("missed deadline by", diff - self.robot_manager.dt)
                continue
            else:
                time.sleep(self.robot_manager.dt - diff)


        if self.args.debug_prints:
            if i < self.max_iterations -1:
                print("success in", i, "iterations!")
            else:
                print("FAIL: did not succed in", max_iterations, "iterations")

        if self.args.real_time_plotting:
            if self.args.debug_prints:
                print("i am putting befree in plotter_queue to stop the real time visualizer")
            self.plotter_queue.put_nowait("befree")
            # just fcks me over when i call again from stopHandler,
            # and that really needs to work
            #self.plotter_queue.close()
            # give it time to become free
            #time.sleep(0.1)
            # TODO: check if this is causing a delay
            self.real_time_plotter_process.terminate()
            if self.args.debug_prints:
                print("terminated real_time_plotter_process")

        # now turn the logs into numpy arrays
        for key in self.log_dict:
            if self.args.debug_prints:
                print("turning log files into numpy arrays")
            self.log_dict[key] = np.array(self.log_dict[key])

        return self.log_dict, self.final_iteration

    def stopHandler(self, signum, frame):
        """
        stopHandler
        -----------
        upon receiving SIGINT it sends zeros for speed commands to
        stop the robot.
        NOTE: apparently this isn't enough,
              nor does stopJ do anything, so it goes to freedriveMode
              and then exits it, which actually stops ur robots at least.
        """
        print('sending 300 speedjs full of zeros and exiting')
        for i in range(300):
            vel_cmd = np.zeros(6)
            #self.robot_manager.rtde_control.speedJ(vel_cmd, 0.1, 1.0 / 500)
            self.robot_manager.sendQd(vel_cmd)
        # hopefully this actually stops it
        if not self.args.pinocchio_only:
            self.robot_manager.rtde_control.speedStop(1)
            print("sending a stopj as well")
            self.robot_manager.rtde_control.stopJ(1)
            print("putting it to freedrive for good measure too")
            self.robot_manager.rtde_control.freedriveMode()

        # set kill command and join visualization processes
        # TODO: actually send them a SIGINT and a SIGKILL if necessary 
        if self.args.real_time_plotting:
            if self.args.debug_prints:
                print("i am putting befree in plotter_queue to stop the real time visualizer")
            self.plotter_queue.put_nowait("befree")
            self.real_time_plotter_process.terminate()
            if self.args.debug_prints:
                print("terminated real_time_plotter_process")

            #self.real_time_plotter_process.kill()
            #self.real_time_plotter_process.terminate()

        if self.args.visualize_manipulator:
            if self.args.debug_prints:
                print("i am putting befree in manipulator to stop the manipulator visualizer")
            self.robot_manager.manipulator_visualizer_queue.put_nowait("befree")
            #time.sleep(1)
            #self.robot_manager.manipulator_visualizer_process.join()
            self.robot_manager.manipulator_visualizer_process.terminate()
            if self.args.debug_prints:
                print("terminated manipulator_visualizer_process")
            #self.robot_manager.manipulator_visualizer_process.kill()
#            self.robot_manager.manipulator_visualizer_process.terminate()
#            if self.args.debug_prints:
#                print("joined manipulator_visualizer_process")
#            self.robot_manager.manipulator_visualizer_process.kill()
#            if self.args.debug_prints:
#                print("joined manipulator_visualizer_process")

        # need to turn logs into ndarrays here too 
        for key in self.log_dict:
            self.log_dict[key] = np.array(self.log_dict[key])
        plotFromDict(self.log_dict, self.final_iteration, self.args)
        
        if not self.args.pinocchio_only:
            self.robot_manager.rtde_control.endFreedriveMode()

        exit()

class RobotManager:
    """
    RobotManager:
    ---------------
    - design goal: rely on pinocchio as much as possible while
                   concealing obvious bookkeeping
    - right now it is assumed you're running this on UR5e so some
      magic numbers are just put to it.
      this will be extended once there's a need for it.
    - at this stage it's just a boilerplate reduction class
      but the idea is to inherit it for more complicated things
      with many steps, like dmp.
      or just showe additional things in, this is python after all
    - you write your controller separately,
      and then drop it into this - there is a wrapper function you put
      around the control loop which handles timing so that you
      actually run at 500Hz and not more.
    - this is probably not the most new-user friendly solution,
      but it's designed for fastest idea to implementation rate.
    - if this was a real programming language, all of these would really be private variables.
      as it currently stands, "private" functions have the '_' prefix 
      while the public getters don't have a prefix.
    - TODO: write out default arguments needed here as well
    """

    # just pass all of the arguments here and store them as is
    # so as to minimize the amount of lines.
    # might be changed later if that seems more appropriate
    def __init__(self, args):
        self.args = args
        self.pinocchio_only = args.pinocchio_only
        self.simulation = args.simulation
        # load model
        # collision and visual models are none if args.visualize == False
        self.model, self.collision_model, self.visual_model, self.data = \
             get_model()
        # we're using meshcat exclusively.
        # there are no good options, 
        # but this does work and isn't a dead project
        if args.visualize_manipulator:
            self.manipulator_visualizer_queue = Queue()
            if args.debug_prints:
                print("ROBOT_MANAGER: i created queue for manipulator visualization:", self.manipulator_visualizer_queue)
                print("ROBOT_MANAGER: i am creating and starting the manipulator visualizer  process")
            self.manipulator_visualizer_process = Process(target=manipulatorVisualizer, 
                                                     args=(self.args, self.model, self.collision_model, self.visual_model, self.manipulator_visualizer_queue, ))
            # give real-time plotter some time to set itself up
            self.manipulator_visualizer_process.start()
            if args.debug_prints:
                print("ROBOT_MANAGER: manipulator_visualizer_process started")
            self.manipulator_visualizer_queue.put(np.zeros(self.model.nq))
            if args.debug_prints:
                print("ROBOT_MANAGER: i managed to put initializing q to manipulator_visualizer_queue")

        # ur specific magic numbers 
        # NOTE: all of this is ur-specific, and needs to be if-ed if other robots are added.
        # TODO: this is 8 in pinocchio and that's what you actually use 
        # if we're being real lmao
        # the TODO here is make this consistent obviously
        self.n_arm_joints = 6
        # last joint because pinocchio adds base frame as 0th joint.
        # and since this is unintuitive, we add the other variable too
        # so that the control designer doesn't need to think about such bs
        self.JOINT_ID = 6
        self.update_rate = 500 #Hz
        self.dt = 1 / self.update_rate
        # you better not give me crazy stuff
        # and i'm not clipping it, you're fixing it
        assert args.acceleration <= 1.7 and args.acceleration > 0.0
        # this is the number passed to speedj
        self.acceleration = args.acceleration
        # NOTE: this is evil and everything only works if it's set to 1
        # you really should control the acceleration via the acceleration argument.
        assert args.speed_slider <= 1.0 and args.acceleration > 0.0
        # TODO: these are almost certainly higher
        # NOTE and TODO: speed slider is evil, put it to 1, handle the rest yourself.
        # NOTE: i have no idea what's the relationship between max_qdd and speed slider
        #self.max_qdd = 1.7 * args.speed_slider
        # NOTE: this is an additional kinda evil speed limitation (by this code, not UR).
        # we're clipping joint velocities with this.
        # if your controllers are not what you expect, you might be commanding a very high velocity,
        # which is clipped, resulting in unexpected movement.
        self.max_qd = 0.5 * args.speed_slider

        self.gripper = None
        if (self.args.gripper != "none") and not self.pinocchio_only:
            if self.args.gripper == "robotiq":
                self.gripper = RobotiqGripper()
                self.gripper.connect(args.robot_ip, 63352)
                self.gripper.activate()
            if self.args.gripper == "onrobot":
                self.gripper = TWOFG()

        # also TODO: figure out how to best solve the gripper_velocity problem
        # NOTE: you need to initialize differently for other types of joints
        self.q = np.zeros(self.model.nq)
        # v_q is the generalization of qd for every type of joint.
        # for revolute joints it's qd, but for ex. the planar joint it's the body velocity.
        self.v_q = np.zeros(self.model.nv)
        # same note as v_q, but it's a_q. 
        self.a_q = np.zeros(self.model.nv)
        # initialize and connect the interfaces
        self.simulation = args.simulation
        if (not args.simulation) and (not args.pinocchio_only) :
            # NOTE: you can't connect twice, so you can't have more than one RobotManager.
            # if this produces errors like "already in use", and it's not already in use,
            # try just running your new program again. it could be that the socket wasn't given
            # back to the os even though you've shut off the previous program.
            print("CONNECTING TO UR5e!")
            self.rtde_control = RTDEControlInterface(args.robot_ip)
            self.rtde_receive = RTDEReceiveInterface(args.robot_ip)
            self.rtde_io = RTDEIOInterface(args.robot_ip)
            # NOTE: the force/torque sensor just has large offsets for no reason,
            # and you need to minus them to have usable readings.
            # we provide this with calibrateFT
            self.wrench_offset = self.calibrateFT()

        elif not args.pinocchio_only:
            self.rtde_control = RTDEControlInterface("127.0.0.1")
            self.rtde_receive = RTDEReceiveInterface("127.0.0.1")
            self.rtde_io = RTDEIOInterface("127.0.0.1")

        self.speed_slider = args.speed_slider
        if not args.pinocchio_only:
            self.rtde_io.setSpeedSlider(args.speed_slider)

        # TODO: make general for other robots
        if args.pinocchio_only and args.start_from_current_pose:
            self.rtde_receive = RTDEReceiveInterface(args.robot_ip)
            q = self.rtde_receive.getActualQ()
            q.append(0.0)
            q.append(0.0)
            q = np.array(q)
            self.q = q
            if args.visualize_manipulator:
                self.manipulator_visualizer_queue.put(q)


        # do it once to get T_w_e
        self._step()

#######################################################################
#               getters which assume you called step()                #
#######################################################################
    
    def getQ(self):
        return self.q.copy()

    def getQd(self):
        return self.v_q.copy()

    def getT_w_e(self, q_given=None):
        if q_given is None:
            return self.T_w_e.copy()
        else:
            assert type(q_given) is np.ndarray
            # calling these here is ok because we rely
            # on robotmanager attributes instead of model.something
            # (which is copying data, but fully separates state and computation,
            # which is important in situations like this)
            pin.forwardKinematics(self.model, self.data, q_given, 
                                  np.zeros(self.model.nv), np.zeros(self.model.nv))
            return self.data.oMi[self.JOINT_ID].copy()


    # this is in EE frame by default (handled in step which
    # is assumed to be called before this)
    def getWrench(self):
        return self.wrench.copy()



    def calibrateFT(self):
        """
        calibrateFT
        -----------
        Read from the f/t sensor a bit, average the results
        and return the result.
        This can be used to offset the bias of the f/t sensor.
        NOTE: this is not an ideal solution.
        ALSO TODO: test whether the offset changes when 
        the manipulator is in different poses.
        """
        ft_readings = []
        print("Will read from f/t sensors for a some number of seconds")
        print("and give you the average.")
        print("Use this as offset.")
        # NOTE: zeroFtSensor() needs to be called frequently because it drifts 
        # by quite a bit in a matter of minutes.
        # if you are running something on the robot for a long period of time, you need
        # to reapply zeroFtSensor() to get reasonable results.
        # because the robot needs to stop for the zeroing to make sense,
        # this is the responsibility of the user!!!
        self.rtde_control.zeroFtSensor()
        for i in range(2000):
            start = time.time()
            ft = self.rtde_receive.getActualTCPForce()
            ft_readings.append(ft)
            end = time.time()
            diff = end - start
            if diff < self.dt:
                time.sleep(self.dt - diff)

        ft_readings = np.array(ft_readings)
        self.wrench_offset = np.average(ft_readings, axis=0)
        print(self.wrench_offset)
        return self.wrench_offset.copy()

    def _step(self):
        """
        _step
        ----
        - the idea is to update everything that should be updated
          on a step-by-step basis
        - the actual problem this is solving is that you're not calling
          forwardKinematics, an expensive call, more than once per step.
        - within the TODO is to make all (necessary) variable private
          so that you can rest assured that everything is handled the way
          it's supposed to be handled. then have getters for these 
          private variables which return deepcopies of whatever you need.
          that way the computations done in the control loop
          can't mess up other things. this is important if you want
          to switch between controllers during operation and have a completely
          painless transition between them.
          TODO: make the getQ, getQd and the rest here do the actual communication,
          and make these functions private.
          then have the deepcopy getters public.
          also TODO: make ifs for the simulation etc.
          this is less ifs overall right.
        """
        self._getQ()
        self._getQd()
        #self._getWrench()
        # computeAllTerms is certainly not necessary btw
        # but if it runs on time, does it matter? it makes everything available...
        # (includes forward kinematics, all jacobians, all dynamics terms, energies)
        # NOTE: it's too slow
        #pin.computeAllTerms(self.model, self.data, self.q, self.v_q)
        pin.forwardKinematics(self.model, self.data, self.q, self.v_q)
        self.T_w_e = self.data.oMi[self.JOINT_ID].copy()
        # wrench in EE should obviously be the default
        self._getWrenchInEE(step_called=True)
        # this isn't real because we're on a velocity-controlled robot, 
        # so this is actually None (no tau, no a_q, as expected)
        self.a_q = self.data.ddq
        # TODO NOTE: you'll want to do the additional math for 
        # torque controlled robots here, but it's ok as is rn

    def setSpeedSlider(self, value):
        """
        setSpeedSlider
        ---------------
        update in all places
        """
        assert value <= 1.0 and value > 0.0
        if not self.args.pinocchio_only:
            self.rtde_io.setSpeedSlider(value)
        self.speed_slider = value
        
    def _getQ(self):
        """
        _getQ
        -----
        NOTE: private function for use in _step(), use the getter getQ()
        urdf treats gripper as two prismatic joints, 
        but they do not affect the overall movement
        of the robot, so we add or remove 2 items to the joint list.
        also, the gripper is controlled separately so we'd need to do this somehow anyway 
        NOTE: this gripper_past_pos thing is not working atm, but i'll keep it here as a TODO
        TODO: make work for new gripper
        """
        if not self.pinocchio_only:
            q = self.rtde_receive.getActualQ()
            if self.args.gripper == "robotiq":
                # TODO: make it work or remove it
                #self.gripper_past_pos = self.gripper_pos
                # this is pointless by itself
                self.gripper_pos = self.gripper.get_current_position()
                # the /255 is to get it dimensionless.
                # the gap is 5cm,
                # thus half the gap is 0.025m (and we only do si units here).
                q.append((self.gripper_pos / 255) * 0.025)
                q.append((self.gripper_pos / 255) * 0.025)
            else:
                # just fill it with zeros otherwise
                q.append(0.0)
                q.append(0.0)
        # let's just have both options for getting q, it's just a 8d float list
        # readability is a somewhat subjective quality after all
            q = np.array(q)
            self.q = q

    # TODO remove evil hack
    def _getT_w_e(self, q_given=None):
        """
        _getT_w_e
        -----
        NOTE: private function, use the getT_w_e() getter
        urdf treats gripper as two prismatic joints, 
        but they do not affect the overall movement
        of the robot, so we add or remove 2 items to the joint list.
        also, the gripper is controlled separately so we'd need to do this somehow anyway 
        NOTE: this gripper_past_pos thing is not working atm, but i'll keep it here as a TODO.
        NOTE: don't use this if use called _step() because it repeats forwardKinematics
        """
        test = True
        try:
            test = q_given.all() == None
            print(test)
            print(q_given)
        except AttributeError:
            test = True

        if test:
            if not self.pinocchio_only:
                q = self.rtde_receive.getActualQ()
                if self.args.gripper:
                    # TODO: make it work or remove it
                    #self.gripper_past_pos = self.gripper_pos
                    # this is pointless by itself
                    self.gripper_pos = self.gripper.get_current_position()
                    # the /255 is to get it dimensionless.
                    # the gap is 5cm,
                    # thus half the gap is 0.025m (and we only do si units here).
                    q.append((self.gripper_pos / 255) * 0.025)
                    q.append((self.gripper_pos / 255) * 0.025)
                else:
                    # just fill it with zeros otherwise
                    q.append(0.0)
                    q.append(0.0)
            else:
                q = self.q
        else:
            q = copy.deepcopy(q_given)
        q = np.array(q)
        self.q = q
        pin.forwardKinematics(self.model, self.data, q)
        # TODO probably remove deepcopy
        self.T_w_e = self.data.oMi[self.JOINT_ID]

    def _getQd(self):
        """
        _getQd
        -----
        NOTE: private function, use the _getQd() getter
        same note as _getQ.
        TODO NOTE: atm there's no way to get current gripper velocity.
        this means you'll probably want to read current positions and then finite-difference 
        to get the velocity.
        as it stands right now, we'll just pass zeros in because I don't need this ATM
        """
        if not self.pinocchio_only:
            qd = self.rtde_receive.getActualQd()
            if self.args.gripper:
                # TODO: this doesn't work because we're not ensuring stuff is called 
                # at every timestep
                #self.gripper_vel = (gripper.get_current_position() - self.gripper_pos) / self.dt
                # so it's just left unused for now - better give nothing than wrong info
                self.gripper_vel = 0.0
                # the /255 is to get it dimensionless
                # the gap is 5cm
                # thus half the gap is 0.025m and we only do si units here
                # no need to deepcopy because only literals are passed
                qd.append(self.gripper_vel)
                qd.append(self.gripper_vel)
            else:
                # just fill it with zeros otherwise
                qd.append(0.0)
                qd.append(0.0)
        # let's just have both options for getting q, it's just a 8d float list
        # readability is a somewhat subjective quality after all
            qd = np.array(qd)
            self.v_q = qd

    def _getWrenchRaw(self):
        """
        _getWrench
        -----
        different things need to be send depending on whether you're running a simulation,
        you're on a real robot, you're running some new simulator bla bla. this is handled
        here because this things depend on the arguments which are manager here (hence the 
        class name RobotManager)
        """
        if not self.pinocchio_only:
            wrench = np.array(self.rtde_receive.getActualTCPForce())
        else:
            raise NotImplementedError("Don't have time to implement this right now.")

    def _getWrench(self):
        if not self.pinocchio_only:
            self.wrench = np.array(self.rtde_receive.getActualTCPForce()) - self.wrench_offset
        else:
            # TODO: do something better here (at least a better distribution)
            self.wrench = np.random.random(self.n_arm_joints)


    def _getWrenchInEE(self, step_called=False):
        if not self.pinocchio_only:
            self.wrench = np.array(self.rtde_receive.getActualTCPForce()) - self.wrench_offset
        else:
            # TODO: do something better here (at least a better distribution)
            self.wrench = np.random.random(self.n_arm_joints)
        if not step_called:
            self._getT_w_e()
        # NOTE: this mapping is equivalent to having a purely rotational action 
        # this is more transparent tho
        mapping = np.zeros((6,6))
        mapping[0:3, 0:3] = self.T_w_e.rotation
        mapping[3:6, 3:6] = self.T_w_e.rotation
        self.wrench = mapping.T @ self.wrench

    def sendQd(self, qd):
        """
        sendQd
        -----
        different things need to be send depending on whether you're running a simulation,
        you're on a real robot, you're running some new simulator bla bla. this is handled
        here because this things depend on the arguments which are manager here (hence the 
        class name RobotManager)
        """
        # we're hiding the extra 2 prismatic joint shenanigans from the control writer
        # because there you shouldn't need to know this anyway
        qd_cmd = qd[:6]
        # np.clip is ok with bounds being scalar, it does what it should
        # (but you can also give it an array)
        qd_cmd = np.clip(qd_cmd, -1 * self.max_qd, self.max_qd)
        if not self.pinocchio_only:
            # speedj(qd, scalar_lead_axis_acc, hangup_time_on_command)
            self.rtde_control.speedJ(qd_cmd, self.acceleration, self.dt)
        else:
            # this one takes all 8 elements of qd since we're still in pinocchio
            # this is ugly, todo: fix
            if len(qd) == 6:
                qd = qd.reshape((6,))
                qd = list(qd)
                qd.append(0.0)
                qd.append(0.0)
                qd = np.array(qd)
            self.q = pin.integrate(self.model, self.q, qd * self.dt)

    def openGripper(self):
        if self.gripper is None:
            if self.args.debug_prints:
                print("you didn't select a gripper (no gripper is the default parameter) so no gripping for you")
            return
        if (not self.args.simulation) and (not self.args.pinocchio_only):
            self.gripper.open()
        else:
            print("not implemented yet, so nothing is going to happen!")

    def closeGripper(self):
        if self.gripper is None:
            if self.args.debug_prints:
                print("you didn't select a gripper (no gripper is the default parameter) so no gripping for you")
            return
        if (not self.args.simulation) and (not self.args.pinocchio_only):
            self.gripper.close()
        else:
            print("not implemented yet, so nothing is going to happen!")

#######################################################################
#                          utility functions                          #
#######################################################################

    def defineGoalPointCLI(self):
        """
        defineGoalPointCLI
        ------------------
        NOTE: this assume _step has not been called because it's run before the controlLoop
        --> best way to handle the goal is to tell the user where the gripper is
            in both UR tcp frame and with pinocchio and have them 
            manually input it when running.
            this way you force the thinking before the moving, 
            but you also get to view and analyze the information first
        TODO get the visual thing you did in ivc project with sliders also.
        it's just text input for now because it's totally usable, just not superb.
        but also you do want to have both options. obviously you go for the sliders
        in the case you're visualizing, makes no sense otherwise.
        """
        self._getQ()
        q = self.getQ()
        # define goal
        pin.forwardKinematics(self.model, self.data, np.array(q))
        T_w_e = self.data.oMi[self.JOINT_ID]
        print("You can only specify the translation right now.")
        if not self.pinocchio_only:
            print("In the following, first 3 numbers are x,y,z position, and second 3 are r,p,y angles")
            print("Here's where the robot is currently. Ensure you know what the base frame is first.")
            print("base frame end-effector pose from pinocchio:\n", \
                    *self.data.oMi[6].translation.round(4), *pin.rpy.matrixToRpy(self.data.oMi[6].rotation).round(4))
            print("UR5e TCP:", *np.array(self.rtde_receive.getActualTCPPose()).round(4))
        # remain with the current orientation
        # TODO: add something, probably rpy for orientation because it's the least number
        # of numbers you need to type in
        Mgoal = T_w_e.copy()
        # this is a reasonable way to do it too, maybe implement it later
        #Mgoal.translation = Mgoal.translation + np.array([0.0, 0.0, -0.1])
        # do a while loop until this is parsed correctly
        while True:
            goal = input("Please enter the target end-effector position in the x.x,y.y,z.z format: ")
            try:
                e = "ok"
                goal_list = goal.split(',')
                for i in range(len(goal_list)):
                   goal_list[i] = float(goal_list[i])
            except:
                e = sys.exc_info()
                print("The input is not in the expected format. Try again.")
                print(e)
            if e == "ok":
                Mgoal.translation = np.array(goal_list)
                break
        print("this is goal pose you defined:\n", Mgoal)

        # NOTE i'm not deepcopying this on purpose
        # but that might be the preferred thing, we'll see
        self.Mgoal = Mgoal
        return Mgoal

    def killManipulatorVisualizer(self):
        """
        killManipulatorVisualizer
        ---------------------------
        if you're using the manipulator visualizer, you want to start it only once.
        because you start the meshcat server, initialize the manipulator and then
        do any subsequent changes with that server. there's no point in restarting.
        but this means you have to kill it manually, because the ControlLoopManager 
        can't nor should know whether this is the last control loop you're running -
        RobotManager has to handle the meshcat server.
        and in this case the user needs to say when the tasks are done.
        """
        if self.args.debug_prints:
            print("i am putting befree in plotter_queue to stop the manipulator visualizer")
        # putting this command tells our process to kill the meshcat zmq server process
        self.manipulator_visualizer_queue.put_nowait("befree")
        time.sleep(0.1)
        self.manipulator_visualizer_process.terminate()
        if self.args.debug_prints:
            print("terminated manipulator_visualizer_process")
