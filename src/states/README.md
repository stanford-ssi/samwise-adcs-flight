# SAMWISE ADCS State Machine

The state machine has the states:
* `init`: We enter this state when we boot, goes straight into `detumble`
* `detumble`: State that runs B-dot to attempt to slow the satellite's rotation. Other states redirect here if the rotation rate is too high.


* `cool_down`: Error state that we enter if something is bad. Gets sensors but does nothing else

## Future states which are currently no-ops
* `nadir_pointing`: State that holds attitude with the camera pointing in the nadir (down) direction
* `desaturating`: State that slows down the reaction wheels. We enter this state if the wheel rotation rate is too high.