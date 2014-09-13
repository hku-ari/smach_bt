import smach
from smach_bt.task import Task, trace_invocations
from smach_bt.container import ContainerTask
from smach_bt.util import log

__all__ = ["ParallelAnyTask", "ParallelAllTask"]

class ParallelTask(ContainerTask):
    def __init__(self, stop_result, continue_result, label=None, input_keys=[], output_keys=[], io_keys=[]):
        ContainerTask.__init__(self, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        
        # Write-once
        self._stop_result = stop_result
        self._continue_result = continue_result
        
        # Mutable
        self._results = None
        self._final_result = None

    def try_immediate(self, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        # With lock
        with self._lock:
            assert(self._runstate == Task.IDLE)
            self._results = [
              self._try_immediate_child(idx, userdata) for idx in range(len(self._tasks))
            ]
            
            # Immediate stop result
            if any([r == self._stop_result for r in self._results]):
                self._cancel_children()
                return self._stop_result
            
            # Immediate continue result
            if all([r == self._continue_result for r in self._results]):
                return self._continue_result
            
            # Deferred result
            self._runstate = Task.READY
            return Task.DEFERRED
        
    def execute_deferred(self, parent_cb, userdata=None):
        if userdata is None:
            userdata = smach.UserData()
        
        # With lock
        with self._lock:
            #log("ParallelTask.execute_deferred()")
            assert(self._runstate == Task.READY)
            self._runstate = Task.EXECUTING
            self._parent_cb = parent_cb
            self._start_children(userdata)
                    
    def cancel(self):
        # With lock
        with self._lock:
            #log("ParallelTask.cancel()")
            assert(self._runstate == Task.READY)
            self._runstate = Task.IDLE
    
    def preempt(self):
        # With lock
        with self._lock:
            #log("ParallelTask.preempt()")
            assert(self._runstate == Task.EXECUTING)
            self._preempt_children()

    # Call with lock held
    def _start_children(self, userdata):
        for idx, r in enumerate(self._results):
            if r == Task.DEFERRED:
                self._execute_deferred_child(idx, self._make_cb(idx), userdata)

    # Call with lock held
    def _cancel_children(self):
        for idx, r in enumerate(self._results):
            if r == Task.DEFERRED:
                self._tasks[idx].cancel()
    
    # Call with lock held
    def _preempt_children(self):
        for idx, r in enumerate(self._results):
            if r == Task.DEFERRED:
                self._tasks[idx].preempt()
    
    def _make_cb(self, idx):
        def cb(result, userdata):
            self._task_termination_cb(result, idx, userdata)
        return cb

    def _task_termination_cb(self, result, task_idx, userdata):
        final_result = None
        cb = None
        # With lock
        with self._lock:
            assert(self._results[task_idx] == Task.DEFERRED)
            self._results[task_idx] = result
            
            # We've not seen a stop result on previous callbacks but we're
            # seeing one now
            if self._final_result is None and result == self._stop_result:
                self._final_result = self._stop_result
                self._preempt_children()
            
            all_done = not any([r == Task.DEFERRED for r in self._results])
            
            if all_done:
                # This is the last callback and they've all been the continue
                # result
                if self._final_result is None and result == self._continue_result:
                    self._final_result = self._continue_result
                
                assert(self._final_result is not None)
                final_result = self._final_result
                cb = self._parent_cb
                self._final_result = None
                self._parent_cb = None
                self._runstate = Task.IDLE
        
        # Without lock
        if cb is not None:
            cb(final_result, userdata)
            
    def _add_active_tasks_recursive(self, label_for_task, active_tasks):
        with self._lock:
            for idx, r in enumerate(self._results):
                if r == Task.DEFERRED:
                    active_task = self._tasks[idx]
                    active_tasks.append(label_for_task[active_task])
                    active_task._add_active_tasks_recursive(label_for_task, active_tasks)
        
# Succeeds if all children succeed
@trace_invocations
class ParallelAllTask(ParallelTask):
    def __init__(self, label=None):
        ParallelTask.__init__(self, stop_result=Task.ABORTED, continue_result=Task.SUCCEEDED, label=label)
        
    def get_label(self):
        if self._label is None:
            return "ParAll"
        return self._label

# Succeeds if any children succeed
@trace_invocations
class ParallelAnyTask(ParallelTask):
    def __init__(self, label=None):
        ParallelTask.__init__(self, stop_result=Task.SUCCEEDED, continue_result=Task.ABORTED, label=label)
    
    def get_label(self):
        if self._label is None:
            return "ParAny"
        return self._label
