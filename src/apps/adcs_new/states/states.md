# States

## State Definitions

Init
This mode just runs through initialization procedures

Safe
This mode does sensor reads and sensor fusion but no controls

ReactionWheelDemo
This mode does sensor reads and fusion and it spins reaction
wheels but without attempting to control.

Detumble
This uses magnetorquers to detumble the satellite.

Nadir
This calculates the nadir point direction and attempts to
control to it.

Test
Fun stuff for whatever we want!

## State Transitions
Init will transition to Safe upon completion.

Safe will transition to Detumble on PiCubed command.

Safe will init to control on picubed command.

