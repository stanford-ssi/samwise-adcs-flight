# ATTITUDE BOARD SOFTWARE SYSTEM

The ADCS system is separated into a set of tasks and a set of states.
The role of the states is to logically dispatch a subset of the tasks.
The role of the tasks is to perform actions on the board. These actions
include everything from software maintenance tasks, communication, 
sensing and control.

## Tasks
The system is implemented using FreeRTOS. This allows us to run confidently
in a multithreaded environment. Different tasks run during each state
and these are controlled using EventGroups. These are essentially a
fancy set of bit flags that enforce some synchronization rules meaning
we can rely on them to be safe. Each state waits until the bit flag is
set before it begins to run. If it attempts to start a loop again once
its set has been disabled, it will block until the bit gets set again.
Each task is associated with a single bit flag and is responsible for
checking whether its own bit has been set. Macros will be written to
help with this.

Additionally, some tasks will want to run only immediately preceeding
other tasks (for example sensor fusion following sensor reading). The
correct way to handle this is with either task notifications or more
likely by just integrating them into a single task.

## State Transitions
State transitions are transmitted using message queues. The state machine
handler is a single thread that waits until it receives a message to run.
Upon receiving a message it will match the message to the current state
and possibly carry out a state transition. For example one task might be
communicating with PiCubed and process commands from PiCubed. If PiCubed
tells it to enter safe mode it will send a message in the message queue
with the information to go to safe mode. This will cause the state machine
handler to unblock. The state machine handler will then note that it is
currently in (for example) control mode, note that the correct transition
when told to go into safe mode directly, and it will implement the correct
state transition to do so. State transitions will always need to turn on or
off the correct event group bits to disable some tasks. They will also often
require some extra code to init or deinit hardware for example.

## Blackboard design pattern
The ADCS system has a number of different tasks that all need to
communicate within each other. The way that we chose to do this is
with a blackboard slate pattern. The blackboard pattern could potentially
lead to race conditions, so this should only be used for data where
slightly incorrect data will not lead to nondeterminism. As far as I'm
aware sensor data should be fine, and state transitions are already safe
because of the message queue. For additional safety, flags can be added
to the slate to indicate whether data is stale or not.

