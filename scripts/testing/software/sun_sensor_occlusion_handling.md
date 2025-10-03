# Sun Sensor Occlusion Analysis

Lundeen Cahilly, 09/17/2025

---

While we can determine the sun vector with just 2 illuminated sun sensors, having only 2 sensors leaves us vulnerable to single-point failures and reduces our measurement accuracy. Due to the geometry of our satellite, sensors that would otherwise illuminate can be occluded by the body of the satellite, creating scenarios where we have minimal redundancy. This warrants investigation because I can already identify cases where we can only illuminate 2 sun sensors, leaving us with no fault tolerance.

To diagnose our blind spots, I put together a blender script that uses basic ray tracing with the satellite model to figure out when a sun sensor is blocked for a given input sun vector. Using this data (format => .csv), the python script in this directory generates spherical plots and statistics on this raytrace data. 

[blender script](https://drive.google.com/file/d/1OyV038isL8jnEkV-aAk7TgyfdYgOgy1_/view?usp=drive_link)

I found via a Monte Carlo simulation (n=100,000) that, for all possible orientations, 6.8% of these orientations leave us with minimal sun sensor redundancy — i.e., 2 or fewer sun sensors are illuminated. In these configurations, a single sensor failure would leave us unable to determine the sun vector reliably.

<iframe src="https://drive.google.com/file/d/1QfC5IYP12uAN9vx8ddhH7L2FsKWy8_2M/preview" width="640" height="480" allow="autoplay"></iframe>

By graphing these vectors as a heatmap on a sphere, where the color indicates the number of sensors illuminated, I show that these 6.8% of "minimal redundancy" orientations (in blue) are caused by the deployables blocking the sun pyramid such that only the Y/Z sensors are illuminated. 

<iframe src="https://drive.google.com/file/d/1a9_U9Mn6mIPCqVAihNxMsGZJobOxzubg/preview" width="640" height="480" allow="autoplay"></iframe>

In the next satellite, we should absolutely fix this blind spot. The graphs show that if we just angled the Y sun sensors to have one point 45º in the +Z direction, and the other 45º in the -Z direction, that this blind spot would be solved. Currently, they point in redundant directions. For now, we'll just have to try to avoid these orientations while doing attitude control.

One possible extension here is to see what happens when we exclude one specific sun sensor (e.g., in the case of one sun sensor failing) and see how this distribution changes. This analysis would be particularly valuable since it would show us the true "blind spots" where sun vector determination becomes impossible, not just difficult or less accurate.