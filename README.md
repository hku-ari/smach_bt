Design Notes for smach_bt

smach_bt is intended to be an implementation of Behavior Trees closely
following the recent ICRA paper in terms of functionality, with the following
additional desirata compared to the implementation described in the paper:

- "Native" ROS action calls available as BT tasks (rather than just used as
protocol with a separate action client required elsewhere)

- Integration with SMACH and the SMACH viewer

- Internally asynchronous rather than polling (almc/behavior_trees) or
  synchronous (smach)

- Explicit preemption rather than preemption by timeout

- If possible, allow structure sharing to work (Behavior DAG rather than tree)

Since the execution is not synchronous, the execute() interface of smach.State
cannot be adhered to, so two wrapper classes are provided for smach
compatibility: smach_bt.BehaviorTreeContainer which wraps a smach_bt.Task and
exposes it as a smach.State, and smach_bt.StateTask which wraps a smach.State
and exposes it as a smach_bt.Task (we call behavior tree nodes 'tasks' rather
'nodes' to avoid confusion with the existing term in ROS)

The desire for the smach_bt.BehaviorTreeContainer work with smach viewer forces a
number of additional constraints: smach_bt.BehaviorTreeContainer is required to
derive from smach.Container, and each task added to smach_bt containers is
required to have a name unique in the tree. This does hurt composability, but
I haven't found a good solution to this yet.

Approaches not taken include:

- Making semantically distinct Condition and Action task types - this would
be fine for leaf tasks, but would hurt composability for containers - if we
make Condition and Action versions of the containers, we then can't compose
Condition and Action tasks in the same container.

- Entirely abandoning the idea of results 'within the same tick' in favour of
explicit timers - this would make preemption behavior hard to predict and
encourage race conditions. This is the approach that would have been required
in order to make each Task follow the smach.State interface, blocking in
execute().

- Allowing multiple invocations of each task in parallel - SMACH doesn't
support this but it could be supported in pure-bt contexts. Right now we store
mutable state in each task that could instead go into a per-invocation object

- Having an internal 'PREEMPTED' result - this is not interesting from the
point of view of bt flow control, so preempted tasks return ABORTED. The
BehaviorTreeContainer will return 'preempted' from execute() if a preemption was
requested.

The interface for each bt Task is:

try_immediate(): returns Task.SUCCEEDED, Task.ABORTED, or Task.DEFERRED. If DEFERRED, task goes into the Task.READY state, otherwise stays IDLE.
cancel(): In READY state, return to IDLE state without starting execution.
execute_deferred(cb): In READY state, starts execution and calls cb on return. Task goes into Task.EXECUTING state.
preempt(): In EXECUTING state, tries to preempt execution. Returns immediately. cb will eventually be called with Task.ABORTED result.

Locking:

- Each smach_bt.Task has a lock
- Parents may call children while parent lock is held
- Children may not call parents (through callbacks) while child lock is held
- Lock is taken when reading or modifying execution state
- Lock is not taken when reading or modifying tree structure - assumed to be
unmodified during execution

Introspection:

The BehaviorTreeContainer implements the smach introspection API. The exposed
transitions do not correspond to the internal transitions of the behavior
tree; instead I chose to give transitions that will cause the tree to be drawn
in the usual way behavior trees are drawn when seen in the smach viewer.
In order for the introspection api to be supported, each task in the
BehaviorTreeContainer needs to have a unique label. If the labels are manually
supplied by the user, this hurts composability, since to combine two arbitrary
trees we need to know that they have no labels in common. I may in future add
the option to have unique labels auto-generated if they are not supplied.

Userdata:

The only place task that persists userdata is the BehaviorTreeContainer; the
bt container tasks (Sequence, etc) just pass through the userdata (with
remappings applied)

Labels:

Since we want to support DAGs and not just trees, and since the smach
introspection API requires that tasks have a unique label, the SMACH model of
labels being assigned to tasks when they are added to a container will not
work, since with a DAG a task can be reachable through multiple containers.
(Actually, the current syntax for assigning labels is probably simply there to
make it easier to put together a state machine without having to assign each
state to a variable, call set_label, and then add it to a container)

Instead, each task can optionally be assigned a label in its initialized or
using the set_label method on the Task itself; if none is supplied then the
task will default to a label based on the task subclass.

As mentioned, the task labels must be unique when supplied to the
introspection API. Putting the burden of ensuring uniqueness on the user would
hurt composability, since the user would need to know when combining two
existing trees that those trees have no labels in common. Instead, the user is
free to pick any labels and reuse labels; when the tree is walked for
introspection, a numerical wart (e.g. _1) is added to task labels as needed
to ensure uniqueness.

Since the tree can be modified at any time, invalidation of a label cache
would be difficult. For now we rebuild the name map every time introspection
methods are called on BehaviorTreeContainer. If this proves too costly in
future, we could give each task a list of watchers that will be notified on
modification, and use this to mark the label cache as dirty.

