import threading

import rospy
import smach
import smach_ros
from smach_bt.task import Task

class BehaviorTreeContainer(smach.Container):
    def __init__(self, input_keys=[], output_keys=[]):
        outcomes = ['succeeded', 'aborted', 'preempted']
        smach.Container.__init__(self, outcomes, input_keys, output_keys)
        self._root = None
        self._initial_task = None
        self._current_exec_root = None
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._result = None
        
        self._task_for_label = {}
        self._label_for_task = {}
        
        # Intentionally public
        self.userdata = smach.UserData()
        
        # Ensures that we halt when task is shut down
        smach_ros.set_preempt_handler(self)
    
    def _rebuild_unique_task_names(self):
        self._task_for_label = {}
        self._label_for_task = {}
        if self._root is not None:
            self._root._add_self_recursive(self._task_for_label, self._label_for_task)
    
    # TODO would like to be able to say 'after set_root has been called, none
    # of the child tasks can be modified' or allow opening/closing with same
    # guarantee when closed. Is there any good way of doing that? Would have
    # to set and track a 'mutable' property on each container.
    
    def set_root(self, task):
        self._root = task
        self._initial_task = self._root
        self.check_consistency()
        
    def _root_termination_cb(self, result, userdata):
        with self._condition:
            if smach.Container.preempt_requested(self):
                result = 'preempted' # HACK
                smach.Container.service_preempt(self)
            
            self._current_exec_root = None
            self._result = result
            self._condition.notify_all()
        
    def _outcome_from_result(self, result):
        if result == Task.SUCCEEDED:
            return 'succeeded'

        if result == Task.ABORTED:
            return 'aborted'
        
        if result == 'preempted': # HACK
           return 'preempted'

        # TODO
        rospy.logwarn("BehaviorTreeContainer: unexpected result %s" % str(result))
        return 'aborted'
    
    def execute(self, parent_ud=smach.UserData()):
        with self._condition:
            if self._root is None:
                raise smach.InvalidTransitionError("BehaviorTreeContainer has no root task")
        
            self._result = None
            self._current_exec_root = self._initial_task
            
            # Copy input keys
            self._copy_input_keys(parent_ud, self.userdata)
            
            result = self._current_exec_root.try_immediate(self.userdata)
        
            if result == Task.DEFERRED:
                self._current_exec_root.execute_deferred(self._root_termination_cb, self.userdata)
                while self._current_exec_root is not None:
                    # Releases condition mutex during wait
                    self._condition.wait(0.1)
                result = self._result
            else:
                self._current_exec_root = None
            
            # Copy output keys
            self._copy_output_keys(self.userdata, parent_ud)
            
            return self._outcome_from_result(result)
        
    def request_preempt(self):
        with self._lock:
            smach.Container.request_preempt(self)
            if self._initial_task is None:
                smach.Container.service_preempt(self)
                return
            self._initial_task.preempt()
            
    # Introspection methods
    
    def get_children(self):
        with self._lock:
            self._rebuild_unique_task_names()
            return self._task_for_label
    
    def set_initial_state(self, initial_states, userdata=None):
        """Set initial active states of a container.
        
        @type initial_states: list of string
        @param initial_states: A description of the initial active state of this
        container.
        
        @type userdata: L{UserData}
        @param userdata: Initial userdata for this container.
        """
        with self._lock:
            if userdata is None:
                userdata = smach.UserData()

            smach.logdebug("Setting initial states to " + str(initial_states))

            if len(initial_states) > 1:
                smach.logwarn("Attempting to set initial state to include more than"
                              " one state, but the BehaviorTreeContainer container can only"
                              " have one initial state. Taking the first one.")

            # Set the initial state label
            if len(initial_states) > 0:
                initial_state_label = initial_states[0]
                # TODO We only support the root as initial state for now
                # for proper support we'll want to walk the tree and set initial idx
                # for every container on the path to the task with the specified
                # label.
                
                self._rebuild_unique_task_names()
                try:
                    self._initial_task = self._task_for_label[initial_state_label]
                except KeyError:
                    smach.logwarn("Unknown initial state '%s' requested", initial_state_label)
            # Set local userdata
            self.userdata.update(userdata)

    def get_initial_states(self):
        with self._lock:
            if self._initial_task is None:
                return []
            self._rebuild_unique_task_names()
            return [self._label_for_task[self._initial_task]]

    def get_active_states(self):
        with self._lock:
            if self._current_exec_root is None:
                return []
            self._rebuild_unique_task_names()
            active_tasks = [self._label_for_task[self._current_exec_root]]
            self._current_exec_root._add_active_tasks_recursive(self._label_for_task, active_tasks)
        return active_tasks

    def get_internal_edges(self):
        with self._lock:
            edges = []
            if self._root is None:
                return edges
            already_visited = set()
            self._rebuild_unique_task_names()
            self._root._add_edges_recursive(self._label_for_task, already_visited, edges)
            # Tree looks nicer without these edges. Maybe render a container with these as external connections?
            # edges.append( ("succeeded", root_label, "succeeded") )
            # edges.append( ("aborted", root_label, "aborted") )
            # edges.append( ("preempted", root_label, "preempted") )
            return edges
    
    def is_running(self):
        """Returns true if the behavior tree is executing."""
        with self._lock:
            return self._current_exec_root is not None
    
    def check_consistency(self):
        self.get_children()
        self.get_initial_states()
        self.get_active_states()
        self.get_internal_edges()
        
    