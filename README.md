# opendrift
all in one controller for micro rc car

status : gyro data fro mpu 6050 over pin a4 a5
put a moving average filter over the gyro signal for making smoother signal 
pull ppm signal from d9 from receiver and map to 0-180 degrees (can change the endpoints here )
and have the gyro data add onto the signal to have gyro stabilisation 
gyrogain can also be set 
added curve so u can set expo , possibility of multiple curves can be explored and put in 

next up , pulling in the throttle signal and processing it so it can have higher startpower and curves on the throttle 
implementing esc using mosfet maybe 

then pcb design and make it small 
