import sys
import threading

import rospy
import smach

from smach_bt.task import Task, trace_invocations

def log(msg):
    # rospy.logwarn(msg)
    print(msg)
    sys.stdout.flush()

class TrueTask(Task):
    def __init__(self, label=None):
        """Initialise a TrueTask.
        """
        Task.__init__(self, label=label)
        
    def get_label(self):
        if self._label is None:
            return "T"
        return self._label
        
    def try_immediate(self, userdata=None):
        with self._lock:
            assert(self._runstate == Task.IDLE)
        return Task.SUCCEEDED

class FalseTask(Task):
    def __init__(self, label=None):
        """Initialise a FalseTask.
        """
        Task.__init__(self, label=label)
    
    def get_label(self):
        if self._label is None:
            return "F"
        return self._label
        
    def try_immediate(self, userdata=None):
        with self._lock:
            assert(self._runstate == Task.IDLE)
        return Task.ABORTED

class PrintTask(Task):
    def __init__(self, msg, label=None):
        Task.__init__(self, label=label)
        self._msg = msg
        
    def try_immediate(self, userdata=None):
        with self._lock:
            assert(self._runstate == Task.IDLE)
        log(self._msg)
        return Task.SUCCEEDED
    
class PrintUserDataTask(Task):
    def __init__(self, output_key, label=None):
        Task.__init__(self, label=label)
        self._output_key = output_key
        
    def try_immediate(self, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
            
        with self._lock:
            assert(self._runstate == Task.IDLE)
        
        try:
            log(userdata[self._output_key])
        except KeyError:
            log("Key not found!")
        
        return Task.SUCCEEDED

# Just for testing really
class SetUserDataTask(Task):
    def __init__(self, value, label=None, key="data"):
        """Initialise a SetUserDataTask.
        """
        Task.__init__(self, label=label, output_keys=[key])
        self._key = key
        self._value = value
        
    def try_immediate(self, userdata=None):
        with self._lock:
            assert(self._runstate == Task.IDLE)
            if userdata is not None:
                userdata[self._key] = self._value
              
        return Task.SUCCEEDED

class SleepTask(Task):
    def __init__(self, sleep_duration, label=None, succeed=True):
        Task.__init__(self, label=label)
        
        # Write-once
        self._sleep_duration = rospy.Duration(sleep_duration) if isinstance(sleep_duration, (int, long, float)) else sleep_duration
        self._sleep_rate = rospy.Rate(1000.0)
        self._result = Task.SUCCEEDED if succeed else Task.ABORTED
        
        # Mutable
        self._preempt_requested = False
        self._parent_cb = None
        
    def try_immediate(self, userdata=None):
        with self._lock:
            # log("SleepTask.try_immediate()")
            assert(self._runstate == Task.IDLE)
            if self._sleep_duration == rospy.Duration(0):
                return self._result
            
            self._runstate = Task.READY
            return Task.DEFERRED
    
    def execute_deferred(self, parent_cb, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
            
        with self._lock:
            # log("SleepTask.execute_deferred()")
            assert(self._runstate == Task.READY)
            assert(self._parent_cb == None)
            
            self._runstate = Task.EXECUTING
            self._parent_cb = parent_cb
            
            start_time = rospy.Time.now()
            self._thread = threading.Thread(
                        name='SleepTask',
                        target=self._thread_fn,
                        args=(start_time,userdata))
            self._thread.start()
    
    def cancel(self):
        with self._lock:
            # log("SleepTask.cancel()")
            assert(self._runstate == Task.READY)
            self._runstate = Task.IDLE
    
    def preempt(self):
        with self._lock:
            # log("SleepTask.preempt()")
            # Need to handle the case where this task has already completed.
            # We can enter preempt() and only after acquiring the lock discover
            # that the execution has completed on its own thread.
            # In fact a task will finish executing before it ever enters the 
            # parent callback!
            if self._runstate == Task.EXECUTING:
                self._preempt_requested = True
            
    def _thread_fn(self, start_time, userdata):
        result = self._result
        while (rospy.Time.now() - start_time < self._sleep_duration):
            if rospy.is_shutdown():
                result = Task.ABORTED
                break
            
            with self._lock:
                if self._preempt_requested:
                    result = Task.ABORTED
                    break
            
            self._sleep_rate.sleep()
        
        cb = None
        with self._lock:
            cb = self._parent_cb
            
            self._preempt_requested = False
            self._runstate = Task.IDLE
            self._parent_cb = None

        cb(result, userdata)

class StateTask(Task):
    def __init__(self, state, label=None, success_outcomes=['success'], should_block=False):
        """Initialise a StateTask.
        
        state: State to wrap as a Task.
        success_outcomes: list of outcomes to be considered successful; any other outcomes will be unsuccessful
        should_block: true if the Task should block until a result is received, rather than returning a Deferred object
        """
        Task.__init__(self, label=label)
        
        # Write-once
        self._state = state
        self._should_block = should_block
        self._success_outcomes = success_outcomes
        self._thread = None
        
        # Mutable
        self._parent_cb = None
    
    def try_immediate(self, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        with self._lock:
            assert(self._runstate == Task.IDLE)
            assert(self._thread == None)
            
            # Clear any previous preempt request
            self._state.service_preempt()
            
            # Want to complete before returning from try_immediate()
            # Could also spawn the thread and join() it, but that doesn't seem necessary
            if self._should_block:
                # Without lock
                self._lock.release()
                return self._execute_state(userdata)
                self._lock.acquire()
                    
            self._runstate = Task.READY
            return Task.DEFERRED
        
    def execute_deferred(self, parent_cb, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        # With lock
        with self._lock:
            assert(self._runstate == Task.READY)
            assert(self._parent_cb == None)
            assert(self._thread == None)
            self._runstate = Task.EXECUTING
            self._parent_cb = parent_cb
            
        # Without lock
        self._thread = threading.Thread(
                name='task:'+self.get_label(),
                target=self._thread_fn,
                args=(userdata,))
        self._thread.start()
    
    def cancel(self):
        with self._lock:
            assert(self._runstate == Task.READY)
            self._runstate = Task.IDLE
        
    def preempt(self):
        # With lock
        assert(self._runstate == Task.EXECUTING)
        self._state.request_preempt()
        
    def _result_from_outcome(self, outcome):
        return Task.SUCCESS if (outcome in self._success_outcomes) else Task.ABORTED
    
    def _thread_fn(self, userdata):
        # Without lock
        result = self._execute_state(userdata)
             
        cb = None
        # With lock
        with self._lock:
            cb = self._parent_cb
            
            self._runstate = Task.IDLE
            self._thread = None
            self._parent_cb = None
        
        # Without lock
        cb(result, userdata)
        
    def _execute_state(self, userdata):
        # Without lock
        outcome = self._state.execute(userdata=userdata)
        
        return self._result_from_outcome(outcome)

class SubscriberTask(Task):
    def __init__(
        self,
        topic_name,
        topic_spec,
        label=None,
        # Receive Policy
        exec_cb=None,
        exec_cb_args=[],
        exec_cb_kwargs={},
        exec_key=None,
        exec_slots=[],
        # Keys
        input_keys=[],
        output_keys=[]
        ):
        
        Task.__init__(self, label=label, input_keys=input_keys, output_keys=output_keys)
        
        self._topic_name = topic_name
        self._topic_spec = topic_spec
        
        self._exec_cb = exec_cb
        self._exec_cb_args = exec_cb_args
        self._exec_cb_kwargs = exec_cb_kwargs
        
        self._exec_cb_input_keys = input_keys
        self._exec_cb_output_keys = output_keys

        self._exec_key = exec_key
        if exec_key is not None:
            self.register_output_keys([exec_key])

        self._exec_slots = exec_slots
        self.register_output_keys(exec_slots)
        
        self._sub = rospy.Subscriber(topic_name, topic_spec, self._recv_cb)
        self._last_msg = None
    
    def _recv_cb(self, msg):
        self._last_msg = msg
        
    def try_immediate(self, userdata):
        # TODO provide a callback on message receipt instead of / as well as on
        # activation?
        
        # TODO allow activation callback to return Task.DEFERRED?
        
        # Call recv callback if it's set
        outcome = None
        if self._exec_cb is not None:
            try:
                outcome = self._exec_cb(
                        smach.Remapper(
                                userdata,
                                self._exec_cb_input_keys,
                                self._exec_cb_output_keys,
                                []),
                        self._last_msg,
                        *self._exec_cb_args,
                        **self._exec_cb_kwargs)
                if outcome not in [Task.ABORTED, Task.SUCCEEDED]:
                    rospy.logerr("Result callback for topic "+self._topic_name+", "+str(self._exec_cb)+" was not registered with the outcomes argument. The exec callback returned '"+str(outcome)+"' but the only registered outcomes are Task.ABORTED andTask.SUCCEEDED")
                    return Task.ABORTED
            except:
                rospy.logerr("Could not execute exec callback: "+traceback.format_exc())
                return Task.ABORTED

        if self._exec_key is not None:
            userdata[self._exec_key] = self._last_msg

        if self._last_msg is not None:
            for key in self._exec_slots:
                userdata[key] = getattr(self._last_msg, key)

        if outcome is None:
            outcome = Task.ABORTED if self._last_msg is None else Task.SUCCEEDED
        
        # On next activation, if we haven't received any new messages, we return
        # ABORTED. This behavior can be changed by the user callback (by
        # stashing the previous message and storing it in userdata if the
        # provided message is None)
        self._last_msg = None
        
        return outcome

# TODO make this a smach_bt.Container?
class Decorator(Task):
    def __init__(self, task, label=None):
        Task.__init__(self, label=label)
        assert(task is not None)
        self._task = task

    def _add_self_recursive(self, task_for_label, label_for_task):
        self._add_self(task_for_label, label_for_task)
        self._task._add_self_recursive(task_for_label, label_for_task)
    
    def _add_active_tasks_recursive(self, label_for_task, active_tasks):
        with self._lock:
            if self._runstate == Task.EXECUTING:
                active_tasks.append(label_for_task[self._task])
                self._task._add_active_tasks_recursive(label_for_task, active_tasks)
    
    def _add_edges_recursive(self, label_for_task, already_visited, edges):
        edges.append( ("", label_for_task[self], label_for_task[self._task]) )
        if self._task not in already_visited:
            already_visited.add(self._task)
            self._task._add_edges_recursive(label_for_task, already_visited, edges)

# Decorate a task so that it succeeds immediately after it has already succeeded
# once.
    
class OnceDecorator(Decorator):
    def __init__(self, task, label=None):
        Decorator.__init__(self, task, label=label)
        self._succeeded = False
        self._parent_cb = None
        self._preempt_requested = False
    
    def get_label(self):
        if self._label is None:
            return "Once"
        return self._label
    
    def try_immediate(self, userdata=None):
        with self._lock:
            assert(self._runstate == Task.IDLE)

            if self._succeeded:
                return Task.SUCCEEDED

            result = self._task.try_immediate(userdata)

            if result == Task.SUCCEEDED:
                self._succeeded = True
                
            if result == Task.DEFERRED:
                self._runstate = Task.READY
                
            return result
                
    def execute_deferred(self, parent_cb, userdata=None):
        with self._lock:
            assert(self._runstate == Task.READY)
            self._runstate = Task.EXECUTING
            self._parent_cb = parent_cb
            self._task.execute_deferred(self._task_termination_cb, userdata)

    def cancel(self):
        with self._lock:
            assert(self._runstate == Task.READY)
            self._task.cancel()
            self._runstate = Task.IDLE
    
    def preempt(self):
        with self._lock:
            if self._runstate == Task.RUNNING:
                self._preempt_requested = True
                self._task.preempt()

    def _task_termination_cb(self, result, userdata):
        cb = None
        # With lock
        with self._lock:
            assert(result != Task.DEFERRED)
            
            cb = self._parent_cb
            self._parent_cb = None
            
            if self._preempt_requested:
                self._preempt_requested = False
                result = Task.ABORTED
            
            if result == Task.SUCCEEDED:
                self._succeeded = True
            else:
                assert(result == Task.ABORTED)
                    
            self._runstate = Task.IDLE
            
        # Without lock
        cb(result, userdata)
