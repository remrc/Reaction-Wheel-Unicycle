# Reaction-Wheel-Unicycle

Gimbal controller BGC 3.1 (use as Arduino), two BLDC gimbal motors GBM2804H-100T, two AS5600 magnetic rotary position sensors, MPU6050, 500mAh LiPo battery.

I use the Simple FOC library to control the BLDC motors.

Balancing controllers can be tuned remotely over bluetooth.

Example (change K1Y):

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

<img src="/pictures/unicycle1.jpg" alt="Unicycle pic"/>
 
More about this:

https://youtu.be/yYkTyglPhxs