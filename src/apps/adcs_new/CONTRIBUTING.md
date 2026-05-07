# ADCS Flight Code Contributing
This is the flight binary for the ADCS subsystem.
The systems are supposed to be separated in the following manner (this
is not always obeyed, but try to obey it as much as possible):

  1. Anything that should be portable between different boards should
  be done outside of this folder. This includes: drivers which should be
  portable between different hardware layouts, gnc algorithms which may
  reference drivers but should not reference hardware.
  2. Anything that references specific hardware should be in this folder.
  You should try to write code to be hardware agnostic such that a reference
  to hardware ammounts to little more than initializing an object or struct
  on slate and then referencing to that object.


## Adding or modifying a state
Almost everything relating to the states is in the [./states/] folder. The
only exception is a state handling task in [./tasks/utility_tasks.cpp]

The different states are enumerated in [./states/states.h]. Reference that
to see what the states are able to be run.

The other main functions are in [./states/states.cpp]. The functions are

    void init_state_machine();

This function inits the states. Most importantly, this is the function
that is responsible for assigning which tasks apply to which state. It
should be pretty obvious from reading the function how to change it.

    void enter_state(StateId_t new_state);

This function does nothing but switch between states. If you want some code
to run whenever a state is switched into, that code should go here. States
should not be changed except by calling this function

    void state_handle_message(StateMsg_t msg);

This matches state messages in order to decide which state to switch to.
All state transitions are done through message passing. This is because
we are in a concurrent environment and we would like some assurances
that there will be no race conditions or the like. This allows every
task to send a message to the state transition task and the state transition
task will just read it and call this function. It is just a double nested
switch statement, and the message types are enumerated in [./states/states.h]

### State Transition Message
For example the following code from vTaskGPS() transitions the state from
a pre-sensor fusion state to sensor fusion. Although it will send the message
regardless of which state the adcs board is in, it will only transition if the
switch statements explicitly say it will.

    if (slate.state_machine.current_state != STATE_FUSION) {
        StateMsg_t msg = MSG_GPS_VALID;
        xQueueSend(slate.state_machine.state_queue_handle,
                &msg, 0);
    }

The state transition task in [./tasks/utility_tasks.cpp] receives from the
queue and calls the function with our switch statements.

## Adding or modifying a task
Everything relating to tasks is in [./tasks/] as well as [./init.cpp]
The [./tasks/tasks.h] enumerates all of the tasks (NTASKS must be at the end).

Every task is a single function that never exists and those are also all
explicitly enumerated in the tasks.h file. The only other place of relevance
is in [./init.cpp] where all of the init functions are crammed into a single
function. This is actually a pretty terrible way of doing this, but it works
reliably and will not lead to any unexpected behaviour. The init function is
always called after the state machine has been initialize in its own task
associated with its own state. Its not recommended to touch this.

One more thing: Each task needs to check that its in the state that allows it
to run. This is done by reading the EventGroupBits (its a FreeRTOS sync
primitive, don't worry about the details). Do, however, worry about forgetting
to use the macro:

    WAIT_UNTIL_EVENTBIT(TASK_BIT(TASK_TASKNAME))

If you forget this, then the task will run regardless of which state you are
in. Bad.

This is really all there is. Just note that this is running in freertos so
there are some macros declared to make it easier to run loops with precise
timing.
