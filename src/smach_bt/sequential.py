import smach
from smach_bt.task import Task
from smach_bt.container import ContainerTask

class SequentialTask(ContainerTask):
    def __init__(self, stop_result, continue_result, label=None, input_keys=[], output_keys=[], io_keys=[]):
        ContainerTask.__init__(self, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        self._initial_task_idx = 0
        self._active_task_idx = None
        self._stop_result = stop_result
        self._continue_result = continue_result
        self._preempt_requested = False
                
    def try_immediate(self, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        # With lock
        with self._lock:
            assert(self._runstate == Task.IDLE)
            assert(self._active_task_idx is None)
            
            (result, self._active_task_idx) = self._loop(self._initial_task_idx, userdata)
            
            if result == Task.DEFERRED:
                self._runstate = Task.READY
            else:
                assert(self._active_task_idx == None)
                self._runstate = Task.IDLE
            
        return result

    def execute_deferred(self, parent_cb, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        with self._lock:
            assert(self._runstate == Task.READY)
            self._runstate = Task.EXECUTING
            self._parent_cb = parent_cb
            self._execute_deferred_child(self._active_task_idx, self._make_cb(self._active_task_idx), userdata)
            
    def cancel(self):
        with self._lock:
            assert(self._runstate == Task.READY)
            assert(self._active_task_idx is not None)
            self._tasks[self._active_task_idx].cancel()
            self._runstate = Task.IDLE
            self._active_task_idx = None
    
    def preempt(self):
        # With lock
        with self._lock:
            # States we can be in when lock is taken/released:
            # Task.IDLE && _active_task_idx is None
            # Task.READY && _active_task_idx is not None
            # Task.EXECUTING && _active_task_idx is not None
            
            if self._runstate == Task.READY:
                assert(self._active_task_idx is not None)
                self.cancel()
                # preemption complete
            elif self._runstate == Task.EXECUTING:
                assert(self._active_task_idx is not None)
                self._tasks[self._active_task_idx].preempt()
                self._preempt_requested = True
                # preemption deferred until callback
            else:
                assert(self._runstate == Task.IDLE)
                assert(self._active_task_idx is None)
                # preemption complete
            
    # Returns (result, final_idx)
    def _loop(self, initial_idx, userdata):
        # self._lock should be locked by calling function.
        # We will stop as soon as any task returns DEFERRED
        # Caller should then call execute_deferred() on that task
        idx = initial_idx
        while idx < len(self._tasks):
            # If self._active_task_idx is not None and initial_idx <= self._active_task_idx,
            # then this _loop call is coming from the timer thread and has reached the active task.
            if idx == self._active_task_idx:
                return (None, idx)
            
            result = self._try_immediate_child(idx, userdata)
        
            if result == Task.DEFERRED:
                return (Task.DEFERRED, idx)
                    
            if result == self._continue_result:
                idx += 1
                continue
            
            if result == self._stop_result:
                return (self._stop_result, None)
        
        return (self._continue_result, None)

    def _make_cb(self, idx):
        def cb(result, userdata):
            self._task_termination_cb(result, idx, userdata)
        return cb

    def _task_termination_cb(self, result, task_idx, userdata):
        cb = None
        # With lock
        with self._lock:
            assert(result != Task.DEFERRED)
            assert(0 <= task_idx and task_idx < len(self._tasks))
            # _active_task_idx might not be the same as the task_idx of the task
            # that just completed, if we were preempted by a timer callback in
            # PreemptingSequenceTask.
            
            cb = self._parent_cb
            
            if self._preempt_requested:
                self._runstate = Task.IDLE
                self._active_task_idx = None
                self._preempt_requested = False
                result = Task.ABORTED
            else:
                if result == self._continue_result:
                    (result, self._active_task_idx) = self._loop(self._active_task_idx + 1, userdata)

                if result == Task.DEFERRED:
                    self._execute_deferred_child(self._active_task_idx, self._make_cb(self._active_task_idx), userdata)
                else:
                    self._runstate = Task.IDLE
                    self._active_task_idx = None
            
        # Without lock
        if result != Task.DEFERRED:
            cb(result, userdata)
            
    def _add_active_tasks_recursive(self, label_for_task, active_tasks):
        with self._lock:
            if self._active_task_idx is not None:
                active_task = self._tasks[self._active_task_idx]
                active_tasks.append(label_for_task[active_task])
                active_task._add_active_tasks_recursive(label_for_task, active_tasks)
            