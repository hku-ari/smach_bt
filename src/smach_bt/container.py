from contextlib import contextmanager
import threading
import traceback
import smach

from smach_bt.task import Task

__all__ = ["ContainerTask"]

class ContainerMixin(object):
    ### TODO Ugh, this is all taken from smach.Container because I want to do the same thing, but this isn't a smach.Container
    
    ### Class members
    _construction_stack = []
    _construction_lock = threading.RLock()
    _context_kwargs = []
    
    ### Context manager methods
    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            return self.close()
        else:
            if exc_type != smach.InvalidStateError and exc_type != smach.InvalidTransitionError:
                smach.logerr("Error raised during SMACH container construction: \n" + "\n".join(traceback.format_exception(exc_type, exc_val, exc_tb)))

    @contextmanager
    def opened(self, **kwargs):
        """Context manager method for opening a container task."""
        self.open()
        prev_kwargs = ContainerMixin._context_kwargs
        ContainerMixin._context_kwargs = kwargs
        try:
            yield self
        finally:
            ContainerMixin._context_kwargs = prev_kwargs 
            self.close()

    def open(self):
        """Opens this container task for modification.

        This appends the container to the construction stack and locks the
        reentrant lock if it is a valid container to open."""

        # Push this container onto the construction stack
        ContainerMixin._construction_stack.append(self)
        ContainerMixin._construction_lock.acquire()

    def close(self):
        """Close the container task."""
        # Make sure this container is the currently open container
        if len(ContainerMixin._construction_stack) > 0:
            if self != ContainerMixin._construction_stack[-1]:
                raise smach.InvalidStateError('Attempting to close a container task that is not currently open.')

        # Pop this container off the construction stack
        ContainerMixin._construction_stack.pop()
        ContainerMixin._construction_lock.release()

        # Check consistency of container, post-construction
        try:
            self.check_consistency()
        except (smach.InvalidStateError, smach.InvalidTransitionError):
            smach.logerr("Container task consistency check failed.")

    def is_opened(self):
        """Returns True if this container is currently opened for construction.
        @rtype: bool
        """
        return len(ContainerMixin._construction_stack) > 0 and self == ContainerMixin._construction_stack[-1]

    def assert_opened(self,msg=''):
        if not self.is_opened():
            raise smach.InvalidConstructionError(msg)
    
    @staticmethod
    def _any_containers_opened():
        """Returns True if any container tasks are opened."""
        if len(ContainerMixin._construction_stack) > 0:
            return True
        return False

    @classmethod
    def _currently_opened_container(cls):
        """Get the currently opened container task.
        
        This also asserts that the open container task is of type cls.
        """
        if ContainerMixin._any_containers_opened():
            opened_container = ContainerMixin._construction_stack[-1]
            if not isinstance(opened_container, cls):
                raise smach.InvalidStateError('Attempting to call a %s construction method without having opened a %s.' % (cls, cls))
            return opened_container
        else:
            raise smach.InvalidStateError('Attempting to access the currently opened container task, but no container task is opened.')
    
    # Do nothing for now
    def check_consistency(self):
        pass


class ContainerTask(Task, ContainerMixin):
    def __init__(self, label=None, input_keys=[], output_keys=[], io_keys=[]):
        """Initialise a Container.
        """
        Task.__init__(self, label=label, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        ContainerMixin.__init__(self)
        self._tasks = []
        self._remappings = []
        self._block_on_preempt = []
    
    def get_children(self):
        return self._tasks
        
    ### Construction methods
    @staticmethod
    def add(task, block_on_preempt=True, remapping=None):
        """Add task to the opened container task.
        
        block_on_preempt:
            - in a parallel container, if we have an immediate result from one
            of our tasks but we started another and have preempted it, then a
            value of True for this parameter will mean we will block until the 
            preemption completes, and a value of False means we will return
            RUNNING and return the result through the parent callback when the
            preemption has completed.
            - in a preempting sequential container, TODO
        """
        
        smach.logdebug('Adding task (%s, %s)' % (task.get_label(), str(task)))
        
        # Get currently opened container
        self = ContainerTask._currently_opened_container()

        # Store task
        self._tasks.append(task)
        self._remappings.append(remapping)
        self._block_on_preempt.append(block_on_preempt)
        
        return task

    # Takes care of the remappings
    def _try_immediate_child(self, idx, userdata):
        assert(userdata is not None)
        task = self._tasks[idx]
        return task.try_immediate(
            smach.Remapper(
                userdata,
                task.get_registered_input_keys(),
                task.get_registered_output_keys(),
                self._remappings[idx]))
                
    def _execute_deferred_child(self, idx, parent_cb, userdata):
        assert(userdata is not None)
        task = self._tasks[idx]
        task.execute_deferred(
            parent_cb,
            smach.Remapper(
                userdata,
                task.get_registered_input_keys(),
                task.get_registered_output_keys(),
                self._remappings[idx]))

    def _add_self_recursive(self, task_for_label, label_for_task):
        self._add_self(task_for_label, label_for_task)
        
        for child in self._tasks:
            child._add_self_recursive(task_for_label, label_for_task)

    def _add_edges_recursive(self, label_for_task, already_visited, edges):
        self_label = label_for_task[self]
        # return [(outcome, label_from, label_to), ...]
        num_children = len(self._tasks)
        for child_idx in range(num_children):
            # Trying something simple first...
            child = self._tasks[child_idx]
            child_label = label_for_task[child]
            edges.append( (str(child_idx), self_label, child_label) )
            
            if child not in already_visited:
                already_visited.add(child)
                child._add_edges_recursive(label_for_task, already_visited, edges)

    # This one needs to be implemented by derived classes
    # _add_active_tasks_recursive(self, label_for_task, active_tasks)
    