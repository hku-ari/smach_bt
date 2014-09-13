import threading
import rospy
import types

# Code taken from State; want to implement same API, but Tasks aren't States
class UserDataMixin(object):
    def __init__(self, input_keys=[], output_keys=[], io_keys=[]):
        # Store userdata interface description
        self._input_keys = set(input_keys + io_keys)
        self._output_keys = set(output_keys + io_keys)
        
    ### Userdata API
    def register_io_keys(self, keys):
        """Add keys to the set of keys from which this task may read and write.
        @type keys: list of strings
        @param keys: List of keys which may be read from and written to when this
        task is active.
        """
        self._input_keys = self._input_keys.union(keys)
        self._output_keys = self._output_keys.union(keys)

    def register_input_keys(self, keys):
        """Add keys to the set of keys from which this task may read.
        @type keys: list of strings
        @param keys: List of keys which may be read from when this task is
        active.
        """
        self._input_keys = self._input_keys.union(keys)

    def get_registered_input_keys(self):
        """Get a tuple of registered input keys."""
        return tuple(self._input_keys)

    def register_output_keys(self, keys):
        """Add keys to the set of keys to which this task may write.
        @type keys: list of strings
        @param keys: List of keys which may be written to when this task is
        active.
        """
        self._output_keys = self._output_keys.union(keys)

    def get_registered_output_keys(self):
        """Get a tuple of registered output keys."""
        return tuple(self._output_keys)

class Task(UserDataMixin):
    # Runstate
    (IDLE, READY, EXECUTING) = ("IDLE", "READY", "EXECUTING")
    
    # Result
    (SUCCEEDED, ABORTED, DEFERRED) = ("SUCCEEDED", "ABORTED", "DEFERRED")
    
    def __init__(self, label=None, input_keys=[], output_keys=[], io_keys=[]):
        """Initialise a Task."""
        UserDataMixin.__init__(self, input_keys, output_keys, io_keys)
        self._lock = threading.Lock()
        self._runstate = Task.IDLE
        self._label = label
        
    def set_label(self, label):
        self._label = label
    
    def get_label(self):
        if self._label is None:
            return self.__class__.__name__
        return self._label
    
    def try_immediate(self, userdata=None):
        """Try to immediately execute a Task.
        
        Should return Task.SUCCEEED, Task.ABORTED or Task.DEFERRED.
        If the execution is to take non-trivial time, or in other words should
        be able to preempt another while running in a preempting container,
        this method should return Task.DEFERRED and set the runstate to READY.
        """
        raise NotImplementedError()
    
    def execute_deferred(self, parent_cb, userdata=None):
        """Start asynchronous execution, calling parent_cb on completion.
        
        Task must be in READY runstate.
        """
        raise NotImplementedError()

    def cancel(self):
        """Cancel asynchronous execution before starting.
        
        Returns to IDLE runstate immediately.
        Task must be in READY runstate.
        """
        raise NotImplementedError()
    
    def preempt(self):
        """Preempt asynchronous exectution after starting.
        
        Callback from start() will eventually be called with Task.ABORTED,
        unless it completes before preemption takes effect.
        Task must be in EXECUTING runstate.
        """
        pass

    def _add_self(self, task_for_label, label_for_task):
        if self in label_for_task:
            # We've already been added - probably through another branch in a DAG
            return
        
        label = self.get_label()
        if label in task_for_label:
            wart_idx = 1
            while (label+"_"+str(wart_idx)) in task_for_label:
                wart_idx += 1
            label = label+"_"+str(wart_idx)

        label_for_task[self] = label
        task_for_label[label] = self

    def _add_self_recursive(self, task_for_label, label_for_task):
        self._add_self(task_for_label, label_for_task)
    
    def _add_active_tasks_recursive(self, label_for_task, active_tasks):
        pass
    
    def _add_edges_recursive(self, label_for_task, already_visited, edges):
        pass
    
def trace_invocations(cls):
    def wrap_try_immediate(inner_try_immediate):
        def wrapped_try_immediate(self, userdata=None):
            label = self.get_label()
            rospy.loginfo("Entering %s" % label)
            result = inner_try_immediate(self, userdata)
            if result != Task.DEFERRED:
                rospy.loginfo("Exiting %s with result %s" % (label, result))
            else:
                rospy.loginfo("%s result deferred" % label)
            return result
        
        return wrapped_try_immediate
        
    def wrap_execute_deferred(inner_execute_deferred):
        def make_cb(self, parent_cb):
            def cb(result, userdata):
                rospy.loginfo("Exiting %s with result %s" % (self.get_label(), result))
                parent_cb(result, userdata)
            return cb
    
        def wrapped_execute_deferred(self, parent_cb, userdata=None):
            inner_execute_deferred(self, make_cb(self, parent_cb), userdata)
            
        return wrapped_execute_deferred

    def wrapper(method_name, method):
        if method_name == "try_immediate":
            return wrap_try_immediate(method)
        if method_name == "execute_deferred":
            return wrap_execute_deferred(method)
        return method
    
    class MetaWrapper(type):
        # Wrap all functions in the class dict
        def __new__(self, cls):
            new_dict = {}
            
            # Wrap methods in resolution order
            for c in cls.mro():
                for attr_name, attr in c.__dict__.items():
                    if attr_name not in new_dict:
                        if type(attr) == types.FunctionType:
                            attr = wrapper(attr_name, attr)
                            new_dict[attr_name] = attr
                            
            # Fill in class attributes only for derived class
            for attr_name, attr in cls.__dict__.items():
                if type(attr) != types.FunctionType:
                    new_dict[attr_name] = attr
                    
            return type.__new__(self, cls.__name__, cls.__bases__, new_dict)
        
        # Support for class attributes (like Container.add)
        def __getattr__(self, key):
            return getattr(cls, key)
    
    # Uncomment to enable logging
    # return MetaWrapper(cls)
    return cls

    