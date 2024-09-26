# STM32F4-Peltier
## Overview and Theory
A proportional integral derivative (PID) controller is a feedback loop designed to bring a process value (PV) to a set point (SP). The error between the PV and the SP is continuously recalculated, and determines how much to adjust a control variable (CV). In this case, the PV and SP is the current and target temperature respectively inside an insulated box, and the control variable is the current supplied to a Peltier device via pulse width modulation (PWM). When the error between the PV and SP is large or if the PV has not reached the SP over a long duration, the CV is adjusted by the controller to pump heat faster; as the PV approaches the SP, the opposite is true. The controller adjusts the CV through three tunable parameters: $K_p,$ $K_i,$ and $K_d.$

By convention, let $SP=r(t),$ $PV=y(t),$ $CV=u(t),$ and $error=e(t).$ Then the mathematical expression of the PID controller is 
$$u(t) = K_pe(t) + K_i \int_0^t e(\tau)d\tau + K_d\frac{de(t)}{dt}.$$
I will tune the three parameters through trial and error to find a combination that quickly cools the temperature to the SP while minimizing overshooting or oscillation about the SP.  

## Hardware

The microcontroller used for this project was an STM32F429I, the Peltier was a TEC1-12706 and its driver was an L298 dual full-bridge, and the various temperature sensors were hand-made using NTC thermistors in a voltage divider (the same as in my solar cell project: https://github.com/ChristopheKanoaMignard/STM32-SolarCell). 

I sandwiched the Peltier between two heat sinks each with a fan and a thermistor embedded inside the heat sink. Each H-bridge can supply $2A$ and the Peltier can handle a max of $6A,$ so we can run them together. The full-bridge leads were shorted together and routed as in the schematic below. 

![FullBridgeWiring](https://github.com/user-attachments/assets/284014e7-fa30-48dc-9757-453fe9333bf5)

At all times, the enables are kept at logic high. Doing so results in the following truth table.
```markdown
| In 1&4 | In 2&3 || Out 1&4 | Out 2&3 || Current direction |
|--------|--------||---------|---------||-------------------|
| 0      | 0      || 0       | 0       || Error             |
| 0      | 1      || 0       | 1       || Reverse (Heating) |
| 1      | 0      || 1       | 0       || Normal (Cooling)  |
| 1      | 1      || 1       | 1       || Error             |
```


## Data and Analysis
To start tuning the controller, I started with the most basic version. I set $K_i=K_d=0$ and only tune $K_p$ across a range of values. Credit to TLK Energy (https://tlk-energy.de/blog-en/practical-pid-tuning-guide) for their blog post describing the tuning process and the example images. 
![tlkEnergyBlog_PidTuning_Kp](https://github.com/user-attachments/assets/3d3d9f83-87d2-4914-9a59-c5852d74fdc2)
|:--:| 
| *Credit: TLK Energy* |
A small $K_p$ will result in a process value asymptoting before it reaches the SP. Progressivley increasing $K_p$ will cause this gap, or remaining control deviation, to shrink, but a good choice of $K_p$ will never cause the PV to reach the SP. Continuing to increase $K_p$ will produce a curve that overshoots the SP, then oscillates about it. 
![PidController_KpTuning_Ki0_Kd0](https://github.com/user-attachments/assets/b0bc164b-df30-4efb-b162-38ff8d55b7c3)

For my system, any $K_p<500$ doesn't get within $1°C$ of the SP and are clearly too small. $K_p = 500, 1000, 2000$ all get pretty close to the SP, but I chose $K_p = 1000$ as my value moving forward. I probably should have chosen $K_p = 2000$ because there is final peak before it settles down to a constant value, just like in the second image from TLK Energy's blog, whereas the curve for $K_p = 1000$ is a smooth curve throughout even if it approaches nearly the same final value at nearly the same time. 
![tlkEnergyBlog_PidTuning_Ki](https://github.com/user-attachments/assets/36ba07ea-01c4-430e-803e-d1f5588b7e9b)
|:--:| 
| *Credit: TLK Energy* |
The TLK blog uses a slightly different formulation of the differentail equation involving $T_I$ in place of $K_i$, but they are simply inversley propotional to each other. So a small $K_i$ will result in a curve that simply looks like a P controller. A good choice will cause PV to slightly overshoot the SP, but then quickly settle down to the SP. A large choice will cause a large overshoot and oscillations about the SP that last for a long time.

For my controller, there are actually two integral related terms that need to be adjusted: $K_i$ and $IMax.$ 
![KiTuning_Kp1000_ISumMax40000_KiVaries](https://github.com/user-attachments/assets/67bebd04-89bb-4870-a13f-fcefd4053b39)
![KiTuning_Kp1000_ISumMaxVaries_KiConstant](https://github.com/user-attachments/assets/1c227ea7-4b88-415f-a26f-ad8a1e5cbf2e)

## Program Features and File Structure
