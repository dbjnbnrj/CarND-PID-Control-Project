##Reflection

The coefficients used in the PID controller are as follows -

Kp: (aka the proportionality coefficient): We need to set the steering angle proportional to the cross track error. This coefficient enables us to correct the steering angle to be proportional to the cross-track error. This enables us to re-adjust to the center of the road if we are steering too frequently to the left or the right. This value is between [-1, 1]

Ki: The integral term will help us realign our wheels. We assume that the simulator wheels are always aligned and that this value is zero.

Kd: This differential term will control our oscillations around the center line.


Here are some of the experiments done with these coefficient values:
| Kp | Ki | Kd  | Description |
| ------------- |:-------------:| -----:|-----:|
| -1 | 0 | 0 | Car oscillates frequently around center. Fails fast.|
| -0.5 | 0 | -0.5 | Car oscillations curbed slightly. Fails fast ( before bridge)|
| -0.5 | 0 | -1.0 | Same as previous case |
| -0.1 | 0 | -0.5 | Improvement over previous case. However oscillations around center are still pretty obvious |
| -0.1 | 0 | -1.0 | Successfully completed round! However turns are not very smooth |
| -0.25 | 0 | -2.5 | Successfully completed round! |


I tried modifying the coefficients at intervals using twiddle but found that the approach was not helpful with the online simulation. 
