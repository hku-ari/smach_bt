import threading

from smach_bt.task import Task, trace_invocations
from smach_bt.sequential import SequentialTask

class PreemptingSequenceTask(SequentialTask):
    def __init__(self, stop_result, continue_result, preempt_rate_hz, label=None, input_keys=[], output_keys=[], io_keys=[]):
        SequentialTask.__init__(self, stop_result=stop_result, continue_result=continue_result, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        self._thread = None
        self._preempt_rate_hz = preempt_rate_hz
        self._condition = threading.Condition(self._lock)
        if preempt_rate_hz > 0:
            self._thread = threading.Thread(_timer_thread_fn)
            self._thread.start()
   
    def _timer_thread_fn(self):
        rate = rospy.Rate(self._preempt_rate_hz)
        while not rospy.is_shutdown():
            # Sleep until active
            with self._condition:
                while self._runstate != Task.EXECUTING:
                    self._condition.wait(1.0)
        
            # Run at fixed rate until no longer active
            while self._timer_cb():
                rate.sleep()
    
    def _timer_cb(self):
        with self._lock:
            if self._runstate != Task.EXECUTING:
                return False
            
            (result, idx) = self._loop(self._initial_task_idx)
            
            if result == None:
                # We reached the active task; we can just return and wait for
                # the next preempt tick
                return False
            
            # If we got this result it would mean we got to the end of the
            # container and execution is complete, so there couldn't have been
            # an active task after all
            assert (result != self._continue_result)

            # In the following two cases, we set some member variables
            # that will be checked in the next callback of the sequence container
            # and cause the correct behaviour (either changing the next task to
            # run, or returning with stop_result)
            
            if result == Task.DEFERRED:
                assert(idx < self._active_task_idx)
                self._tasks[self._active_task_idx].preempt()
                self._active_task_idx = idx
                return True
            
            if result == self._stop_result:
                assert(idx is None)
                self._tasks[self._active_task_idx].preempt()
                self._preempt_requested = True # not great, but will cause Task.ABORTED result in next callback
                return True

@trace_invocations
class SequenceTask(PreemptingSequenceTask):
    def __init__(self, preempt_rate_hz=0, label=None, input_keys=[], output_keys=[], io_keys=[]):
        PreemptingSequenceTask.__init__(self, stop_result=Task.ABORTED, continue_result=Task.SUCCEEDED, preempt_rate_hz=preempt_rate_hz, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
    
    def get_label(self):
        if self._label is None:
            if self._preempt_rate_hz > 0:
                return "SeqAll(P)"
            else:
                return "SeqAll"
        return self._label

@trace_invocations
class SelectorTask(PreemptingSequenceTask):
    def __init__(self, preempt_rate_hz=0, label=None, input_keys=[], output_keys=[], io_keys=[]):
        PreemptingSequenceTask.__init__(self, stop_result=Task.SUCCEEDED, continue_result=Task.ABORTED, preempt_rate_hz=preempt_rate_hz, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
    
    def get_label(self):
        if self._label is None:
            if self._preempt_rate_hz > 0:
                return "SeqAny(P)"
            else:
                return "SeqAny"
        return self._label