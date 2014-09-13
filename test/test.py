#!/usr/bin/env python

PKG = 'smach_bt'
import roslib; roslib.load_manifest(PKG)

import threading

import rospy
import unittest
import sys

import smach
import smach_ros
import smach_bt
from smach_bt import log, Task, TrueTask, FalseTask, SleepTask, SequenceTask, SelectorTask, ParallelAnyTask, ParallelAllTask, OnceDecorator, StateTask
from smach_bt.util import SetUserDataTask

class CompletionRecorder(object):
    def __init__(self, task):
        self.task = task
        self.seen_result = None
        self.end_time = None
           
    def cb(self, result, userdata):
        self.end_time = rospy.Time.now()
        self.seen_result = result
    
    def has_completed(self):
        return self.end_time is not None
    
    def assert_completes(self, test_case, result, min_time, max_time):
        start_time = rospy.Time.now()

        while not self.has_completed():
            if (rospy.Time.now() - start_time).to_sec() > max_time:
                test_case.assertTrue(False, "Wait timed out!")

            rospy.sleep(0.1) # TODO should be based on how long we expect to wait but good enough for now

        duration = (self.end_time - start_time).to_sec()

        test_case.assertEqual(self.seen_result, result)
        test_case.assertGreaterEqual(duration, min_time)
        test_case.assertLessEqual(duration, max_time)

def assert_immediate_result(test_case, task, result):
    test_case.assertEqual(task.try_immediate(), result)
    
def assert_async_result(test_case, task, result, expected_time):
    test_case.assertEqual(task.try_immediate(), Task.DEFERRED)
    cr = CompletionRecorder(task)
    task.execute_deferred(cr.cb)
    cr.assert_completes(test_case, result, expected_time * 0.8, expected_time * 1.2)

def assert_preempts(test_case, task, max_time):
    test_case.assertEqual(task.try_immediate(), Task.DEFERRED)
    cr = CompletionRecorder(task)
    task.execute_deferred(cr.cb)
    task.preempt()
    cr.assert_completes(test_case, Task.ABORTED, 0.0, max_time)

def assert_result(test_case, task, result):
    if isinstance(result, tuple):
        assert_async_result(test_case, task, result[0], result[1])
    else:
        assert_immediate_result(test_case, task, result)

def make_task(task_spec):
    if task_spec == 'f':
        return FalseTask()
    if task_spec == 't':
        return TrueTask()
    if task_spec == 'af':
        return smach_bt.SleepTask(rospy.Duration(0.1), succeed=False)
    if task_spec == 'at':
        return smach_bt.SleepTask(rospy.Duration(0.1), succeed=True)
    raise RuntimeError("welp")

def make_collection(cls, col_spec):
    if col_spec == 'empty':
        return cls()
    if col_spec == 'preempt':
        return make_collection(cls, 'at;af')
    
    s = cls()
    with s:
        idx = 0
        for ns in col_spec.split(';'):
            cls.add(make_task(ns))
            idx += 1
    return s

# list format: [(key, value)]
# keys: 'empty', 'preempt', 'f', 't', 'f;f', 'f;t', 't;f', 't;t', 'f;af', 'f;at', 't;af', 't;at', 'af;f', 'af;t', 'at;f', 'at;t', 'af', 'at', 'af;af', 'af;at', 'at;af', 'at;at'
# values:
#   int: assert immediate result
#   tuple: if preempt, assert deadline; otherwise value is +/- 20%)
def test_collection(test_case, cls, test_list):
    for (spec, result) in test_list:
        task = make_collection(cls, spec)
        for run_num in [1, 2]: # Run twice to ensure the state of the task after completion is correct
            print("spec: " + str(cls) + " " + spec + " run " + str(run_num))
            sys.stdout.flush()
            if spec == "preempt":
                assert_preempts(test_case, task, result[1])
            else:
                assert_result(test_case, task, result)
            assert(task._runstate == Task.IDLE)

class TestSimpleTasks(unittest.TestCase):
    def test_task(self):
        n = Task()
        self.assertRaises(NotImplementedError, n.try_immediate)
    
    def test_true(self):
        n = TrueTask()
        assert_immediate_result(self, n, Task.SUCCEEDED)
        
    def test_false(self):
        n = FalseTask()
        assert_immediate_result(self, n, Task.ABORTED)
        
    def test_sleep_1sec(self):
        n = smach_bt.SleepTask(rospy.Duration(1.0))
        assert_async_result(self, n, Task.SUCCEEDED, 1.0)
        
    def test_sleep_preempt(self):
        n = smach_bt.SleepTask(rospy.Duration(10000.0))
        assert_preempts(self, n, 1.0)
    
    # TODO test output?
    def test_print(self):
        n = smach_bt.PrintTask("Testing print.")
        assert_immediate_result(self, n, Task.SUCCEEDED)

class TestCollections(unittest.TestCase):
    def test_sequence(self):
        test_collection(self, SequenceTask, [
            ('empty', Task.SUCCEEDED),
            ('preempt', (Task.ABORTED, 1.0)),
            
            ('f', Task.ABORTED),
            ('t', Task.SUCCEEDED),
            
            ('af', (Task.ABORTED, 0.1)),
            ('at', (Task.SUCCEEDED, 0.1)),
            
            ('f;f', Task.ABORTED),
            ('f;t', Task.ABORTED),
            ('t;f', Task.ABORTED),
            ('t;t', Task.SUCCEEDED),
            
            ('f;af', Task.ABORTED),
            ('f;at', Task.ABORTED),
            ('t;af', (Task.ABORTED, 0.1)),
            ('t;at', (Task.SUCCEEDED, 0.1)),
            
            ('af;f', (Task.ABORTED, 0.1)),
            ('af;t', (Task.ABORTED, 0.1)),
            ('at;f', (Task.ABORTED, 0.1)),
            ('at;t', (Task.SUCCEEDED, 0.1)),
            
            ('af;af', (Task.ABORTED, 0.1)),
            ('af;at', (Task.ABORTED, 0.1)),
            ('at;af', (Task.ABORTED, 0.2)),
            ('at;at', (Task.SUCCEEDED, 0.2))
        ])
    
    def test_selector(self):
        test_collection(self, SelectorTask, [
            ('empty', Task.ABORTED),
            ('preempt', (Task.ABORTED, 1.0)),
            
            ('f', Task.ABORTED),
            ('t', Task.SUCCEEDED),
            
            ('af', (Task.ABORTED, 0.1)),
            ('at', (Task.SUCCEEDED, 0.1)),
            
            ('f;f', Task.ABORTED),
            ('f;t', Task.SUCCEEDED),
            ('t;f', Task.SUCCEEDED),
            ('t;t', Task.SUCCEEDED),
            
            ('f;af', (Task.ABORTED, 0.1)),
            ('f;at', (Task.SUCCEEDED, 0.1)),
            ('t;af', Task.SUCCEEDED),
            ('t;at', Task.SUCCEEDED),
            
            ('af;f', (Task.ABORTED, 0.1)),
            ('af;t', (Task.SUCCEEDED, 0.1)),
            ('at;f', (Task.SUCCEEDED, 0.1)),
            ('at;t', (Task.SUCCEEDED, 0.1)),
           
            ('af;af', (Task.ABORTED, 0.2)),
            ('af;at', (Task.SUCCEEDED, 0.2)),
            ('at;af', (Task.SUCCEEDED, 0.1)),
            ('at;at', (Task.SUCCEEDED, 0.1))
        ])
    
    def test_parallel_all(self):
        test_collection(self, ParallelAllTask, [
            ('empty', Task.SUCCEEDED),
            #('preempt', (Task.ABORTED, 1.0)),
            
            ('f', Task.ABORTED),
            ('t', Task.SUCCEEDED),
            
            ('af', (Task.ABORTED, 0.1)),
            ('at', (Task.SUCCEEDED, 0.1)),
            
            ('f;f', Task.ABORTED),
            ('f;t', Task.ABORTED),
            ('t;f', Task.ABORTED),
            ('t;t', Task.SUCCEEDED),
            
            ('f;af', Task.ABORTED),
            ('f;at', Task.ABORTED),
            ('t;af', (Task.ABORTED, 0.1)),
            ('t;at', (Task.SUCCEEDED, 0.1)),
            
            ('af;f', Task.ABORTED),
            ('af;t', (Task.ABORTED, 0.1)),
            ('at;f', Task.ABORTED),
            ('at;t', (Task.SUCCEEDED, 0.1)),
            
            ('af;af', (Task.ABORTED, 0.1)),
            ('af;at', (Task.ABORTED, 0.1)),
            ('at;af', (Task.ABORTED, 0.1)),
            ('at;at', (Task.SUCCEEDED, 0.1))
        ])
        
    def test_parallel_any(self):
        test_collection(self, ParallelAnyTask, [
            ('empty', Task.ABORTED),
            #('preempt', (Task.ABORTED, 1.0)),
            
            ('f', Task.ABORTED),
            ('t', Task.SUCCEEDED),
            
            ('af', (Task.ABORTED, 0.1)),
            ('at', (Task.SUCCEEDED, 0.1)),
            
            ('f;f', Task.ABORTED),
            ('f;t', Task.SUCCEEDED),
            ('t;f', Task.SUCCEEDED),
            ('t;t', Task.SUCCEEDED),
            
            ('f;af', (Task.ABORTED, 0.1)),
            ('f;at', (Task.SUCCEEDED, 0.1)),
            ('t;af', Task.SUCCEEDED),
            ('t;at', Task.SUCCEEDED),
            
            ('af;f', (Task.ABORTED, 0.1)),
            ('af;t', Task.SUCCEEDED),
            ('at;f', (Task.SUCCEEDED, 0.1)),
            ('at;t', Task.SUCCEEDED),
            
            ('af;af', (Task.ABORTED, 0.1)),
            ('af;at', (Task.SUCCEEDED, 0.1)),
            ('at;af', (Task.SUCCEEDED, 0.1)),
            ('at;at', (Task.SUCCEEDED, 0.1))
        ])

# TODO tests for non-0 preempt timer on Sequence/Selector tasks
# TODO tests for StateTask

class TestBehaviorTree(unittest.TestCase):
    def test_empty(self):
        bt = smach_bt.BehaviorTreeContainer()
        self.assertRaises(smach.InvalidTransitionError, bt.execute)

    def test_true(self):
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(TrueTask())
        self.assertEqual(bt.execute(), 'succeeded')
    
    def test_false(self):
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(FalseTask())
        self.assertEqual(bt.execute(), 'aborted')
    
    def test_run_true(self):
        seq = SequenceTask()
        with seq:
            SequenceTask.add(SleepTask(0.1))
            SequenceTask.add(TrueTask())
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(seq)
        self.assertEqual(bt.execute(), 'succeeded')
    
    def test_run_false(self):
        seq = SequenceTask()
        with seq:
            SequenceTask.add(SleepTask(0.1))
            SequenceTask.add(FalseTask())
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(seq)
        self.assertEqual(bt.execute(), 'aborted')
        
    def _preempt_thread(self, bt):
        rospy.sleep(rospy.Duration(0.1)) # TODO implement better? could use a bt with two tasks in seq, first of which sets a var and second of which sleeps
        bt.request_preempt()
        
    def test_preempt(self):
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(SleepTask(1000))
        thread = threading.Thread(
                name='test_preempt',
                target=self._preempt_thread,
                args=(bt,))
        thread.start()
        self.assertEqual(bt.execute(), 'preempted')

class TestUserData(unittest.TestCase):
    def test_set(self):
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(SetUserDataTask(key="data", value="test"))
        self.assertEqual(bt.execute(), 'succeeded')
        self.assertEqual(bt.userdata["data"], "test")

    def test_remap(self):
        bt = smach_bt.BehaviorTreeContainer()
        seq = smach_bt.SequenceTask(output_keys=['test_key'])
        with seq:
            smach_bt.SequenceTask.add(SetUserDataTask(key="data", value="test"), remapping={'data': 'test_key'})
        bt.set_root(seq)
        self.assertEqual(bt.execute(), 'succeeded')
        self.assertEqual(bt.userdata["test_key"], "test")

class TestIntrospection:#(unittest.TestCase):
    def test_sequence(self):
        p = ParallelAnyTask()
        with p:
            ParallelAnyTask.add(TrueTask())
            ParallelAnyTask.add(FalseTask())
            
        s = SequenceTask()
        with s:
            SequenceTask.add(p)
            SequenceTask.add(FalseTask())
            
        p2 = ParallelAnyTask()
        with p2:
            ParallelAllTask.add(s)
            ParallelAllTask.add(SleepTask(1.0, succeed=False))
            
        s2 = SelectorTask()
        with s2:
            SelectorTask.add(p2)
            SelectorTask.add(SleepTask(1.0))
            
        bt = smach_bt.BehaviorTreeContainer()
        bt.set_root(s2)
        sis = smach_ros.IntrospectionServer('server_name', bt, '/SM_ROOT')
        sis.start()
        wait_start = rospy.Time.now()
        while (rospy.Time.now() - wait_start).to_sec() < 10 and not rospy.is_shutdown():
            rospy.sleep(0.1)
            rospy.loginfo("Executing...")
            outcome = bt.execute()

if __name__ == '__main__':
    rospy.init_node('smach_bt_test', disable_signals=False)
    
    unittest.main(verbosity=4)
    
