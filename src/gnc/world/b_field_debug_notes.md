## Creating this file to document our debug progress!!

# Debug!!

Thursday June 5:

Lundeen, I(Carson) and Niklas worked on finding bugs in the code.

Good news!! Both Niklas and I found some.

The bug I found was in the b_field generating function.
Basically in the function to generate the b fields as a summation of spherical harmonics with some factors in front.
The summation is over both n and m, ranging from 0 < n <= max_order and 0 <= m <= n.
Basically the issue was that one of the factors was being multiplied inside of a summation.
*I no longer think this was an actual issue because it literally just distributes and its fine*


Niklas's Bug:



