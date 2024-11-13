# STM32F4-Peltier
## Overview and Theory
A proportional integral derivative (PID) controller is a feedback loop designed to bring a process value (PV) to a set point (SP). The error between the PV and the SP is periodically recalculated in order to determine how much to adjust a control variable (CV). In this case, the PV and SP is the current and target temperature respectively inside an insulated box, and the control variable is the current supplied to a Peltier device via pulse width modulation (PWM). When the error between the PV and SP is large or if the PV has not reached the SP over a long duration, the CV is adjusted by the controller to pump heat faster; as the PV approaches the SP, the opposite is true. The controller adjusts the CV through three tunable parameters: $K_p,$ $K_i,$ and $K_d.$

By convention, let $SP=r(t),$ $PV=y(t),$ $CV=u(t),$ and $error=e(t).$ Then the mathematical expression of the PID controller is 
$$u(t) = K_pe(t) + K_i \int_0^t e(\tau)d\tau + K_d\frac{de(t)}{dt}.$$
I will tune the three parameters through trial and error to find a combination that quickly cools the temperature to the SP while minimizing overshooting or oscillation about the SP.  

## Hardware

The microcontroller used for this project was an STM32F429I, the Peltier was a TEC1-12706 and its driver was an L298 dual full-bridge, and the various temperature sensors were hand-made using NTC thermistors in a voltage divider (the same as in my solar cell project: https://github.com/ChristopheKanoaMignard/STM32-SolarCell). 

I sandwiched the Peltier between two heat sinks each with a fan and a thermistor embedded inside the heat sink. Each H-bridge can supply $2A$ for a maximum current of $4A$ and the Peltier can handle up to $6A,$ so we can run them together. The full-bridge leads were shorted together and routed as in the schematic below. 

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

When the current is directed to run in cooling mode, the duty cycle of the PWM is equivalent to the percent of maximum current supplied to the Peltier. So a duty cycle of 75% will supply $0.75*2*2A=3A$ of current. When the inputs are changed to reverse the current direction to instead heat the Peltier, the current supplied is proportional to $1-duty cycle$, so a duty cycle of 25% will supply $(1-0.25)*2*2A=3A.$ Therefore the PWM duty cycle is the PID control variable since we can easily change its value, and it controls the system temperature i.e. the process variable. 

A note of caution: while the Peltier can safley handle the $4A$ maximum current, at this current the heat sink on the L298 rapidly gets too hot to touch, even when a cooling fan was placed on it. While a more rigorous solution would be to attach a thermistor and interlock to the heat sink, I determined through trial and error that the heat sink remains at a reasonably cool temperature when the system is run indefinitley at $3A.$ Therefore my PID controller has a cap ensuring that the PWM never exceeds 75% duty cycle in cooling mode or drops below 25% in heating mode. 
```ruby
if (Regs.s32[RegPeltierPidMv] > pwmCcrMaxCool) {		
	pwmCcr = pwmCcrMaxCool;										//Maximum allowed cooling. By default, pwmCcrMaxCool=0.75*0xFFFF
}
else {
	pwmCcr = (uint16_t) Regs.s32[RegPeltierPidMv];				//Partial cooling
}
```


## Data and Analysis
To start tuning the controller, I started with the most basic version. I set $K_i=K_d=0$ and only tune $K_p$ across a range of values. Credit to TLK Energy (https://tlk-energy.de/blog-en/practical-pid-tuning-guide) for their blog post describing the tuning process and the example images. 

![tlkEnergyBlog_PidTuning_Kp](https://github.com/user-attachments/assets/3d3d9f83-87d2-4914-9a59-c5852d74fdc2)
|:--:| 
| *Credit: TLK Energy* |

A small $K_p$ will result in a process value asymptoting before it reaches the SP. Progressivley increasing $K_p$ will cause this gap, or remaining control deviation, to shrink, but a good choice of $K_p$ will never cause the PV to reach the SP. Continuing to increase $K_p$ will produce a curve that overshoots the SP, then oscillates about it. 

![PidController_KpTuning_Ki0_Kd0](https://github.com/user-attachments/assets/b0bc164b-df30-4efb-b162-38ff8d55b7c3)

For my system, any $K_p<500$ doesn't get within $1°C$ of the SP and are clearly too small. $K_p = 500, 1000, 2000$ all get pretty close to the SP, but I chose $K_p = 1000$ as my value moving forward. I probably should have chosen $K_p = 2000$ because there is peak that doesn't reach the SP but does overshoot its steady state temperature, just like in the second image from TLK Energy's blog, whereas the curve for $K_p = 1000$ is a smooth curve throughout even if it approaches nearly the same final value at nearly the same time. However, with a system as imprecise as this cooler, the difference between these options will be negligable on overall impact. 

![tlkEnergyBlog_PidTuning_Ki](https://github.com/user-attachments/assets/36ba07ea-01c4-430e-803e-d1f5588b7e9b)
|:--:| 
| *Credit: TLK Energy* |

The TLK blog uses a slightly different formulation of the differentail equation involving $T_I$ in place of $K_i$, but they are simply inversley propotional to each other. So a small $K_i$ will result in a curve that simply looks like a P controller. A good choice will cause PV to slightly overshoot the SP, but then quickly settle down to the SP. A large choice will cause a large overshoot and oscillations about the SP that last for a long time.

For my controller, there are actually two integral related terms that need to be adjusted: $K_i$ and $IMax.$ Starting with $K_i$, as expected, increasing its value had a negligable impact on rise time; instead, it imparts a momentum to the system so that the CV can actually reach and exceed the SP. My results differ slightly from the TLK blog because my integral term has an additional degree of freedom in $ISumMax$, a variable in the code that places an upper limit on the integral term's contribution to the MV. Between the trials below, the best result was achieved by setting $K_i$ to $500$ when $ISumMax$ is $40,000$ because the CV overshoots the SP, then rapidly corrects iself with minimal overcorrection, and these overcorrection oscillations quickly attenuate. During the tuning process I found that while choosing good $K_i$ and $ISumMax$ is important, it is equally important to choose a good ratio between the two.

Increasing $K_i$ makes the system more responsive so that for good values the CV slightly overshoots the SP, maybe oscillates around it once or twice, then settles down. For overly large $K_i$ the system is actually too responsive and constantly overcorrecting, resulting in perpetual oscillation around the SP, while at small $K_i$ the system looks just like a P controller, except the steady state temperature is actually beyond the SP due to the influence of $ISumMax$.

![KiTuning_Kp1000_ISumMax40000_KiVaries](https://github.com/user-attachments/assets/67bebd04-89bb-4870-a13f-fcefd4053b39)

Wheras $K_i$ controls the responsiveness of the system, $ISumMax$ is the momentum. To display the effects of $ISumMax$, I elected to use a bad choice of $K_i$ since the effect is easier to observe. A good choice of $ISumMax$ will reach the SP very quickly while limiting the overshoot. A large choice may not look bad initially, but when $K_i$ is tuned, it must be very large to make the system responsive to change which can cause instability. Maintaining the analogy of momentum, tuning $K_i$ for large $ISumMax$ is like trying to make a right hand turn in a car driving at highway speeds: possible but unstable. Additionally, such a system will be very bad at reaching SP very different from the one used in tuning because of this inherent instability. Having $ISumMax$ too small makes the risetime too long, and no changes to $K_i$ will have a noticeable effect. 

![KiTuning_Kp1000_ISumMaxVaries_KiConstant](https://github.com/user-attachments/assets/1c227ea7-4b88-415f-a26f-ad8a1e5cbf2e)

Although the D term is implemented and I took data for it, the effect was wholly negligable. This was to be expected, partially because for good tuning parameters the oscillations did not last long. I'd enjoy the added complexity of tuning an additional degree of freedom, so I may try to find a different experimental apparatus that actually calls for a D term to be included. 

## Program Features and File Structure

The digital PID controller is located in fw\src\main.c, in a function called PidControlCalculation and reproduced below.

```ruby
void PidControlCalculation(void)	
{
	static int32_t errorValue = 0, lastErrorValue = 0;
	static uint16_t pwmCcr = 0;						//Set the pwm capture compare register to this value
	const static uint16_t pwmCcrMaxCool = (uint16_t) (0.75 * 0xFFFF);		//Theoretical max the pwm CCR can hold is 0xFFFF. However, this much current will quickly overheat the driver board heat sink. For now, simply cap the max pwm duty cycle: 50% keeps the heatsink from getting hot to the touch. Should replace this with a temperature sensative interlock. 
	const static uint16_t pwmCcrMaxHeat = 0xFFFF - pwmCcrMaxCool;			//If max cooling is .75*0xFFFF, then max heating should be 0xFFFF - (.75*0xFFFF)
	
	errorValue = Regs.u16[RegPeltierPidSetpointAdc] - Regs.u16[RegPeltierTempColdsideAdc];
	
	//Temporarily store summation in variable rather than reg in case data is read by gui and |integralSummation| > Regs.u16[RegPeltierPidISumMax] but before the maximum gets enforced. Unlikely but theoretically possible
	integralSummation = integralSummation + Regs.u16[RegPeltierPidKi] * errorValue * (0.001*Regs.u16[RegPeltierPidDt]);		//Integral windup is handled by SetReg(RegPeltierPidSetpointAdc). Convert dt from ms to s
	if (integralSummation > Regs.s32[RegPeltierPidISumMax]) {
		integralSummation = Regs.s32[RegPeltierPidISumMax];
	}
	else if (integralSummation < (-1 * Regs.s32[RegPeltierPidISumMax])) {
		integralSummation = -1 * Regs.s32[RegPeltierPidISumMax];
	}
	Regs.s32[RegPeltierPidISum] = integralSummation;
	
	Regs.s32[RegPeltierPidMv] =	(int64_t)(Regs.u16[RegPeltierPidKp] * errorValue + 
											Regs.s32[RegPeltierPidISum] + 
											Regs.u16[RegPeltierPidKd] * (errorValue - lastErrorValue) / Regs.u16[RegPeltierPidDt]);
	lastErrorValue = errorValue;
		
	if (Regs.s32[RegPeltierPidMv] >= 0)		//If MV is positive or 0, then peltier should be in normal mode and maximal cooling is 100% duty
	{	
		SetReg(RegPeltierReverseMode, 0);				//Run peltier in normal mode ie cooling. 
		if (Regs.s32[RegPeltierPidMv] > pwmCcrMaxCool) {		
			pwmCcr = pwmCcrMaxCool;							//Maximum allowed cooling.
		}
		else {
			pwmCcr = (uint16_t) Regs.s32[RegPeltierPidMv];				//Partial cooling
		}
	}
	else							//If MV is negative, then peltier should be in reverse mode and maximal heating is 0% duty cycle
	{
		SetReg(RegPeltierReverseMode, 1);					//Run peltier in reverse mode.
			
		//Regs.s32[RegPeltierPidMv] = Regs.s32[RegPeltierPidMv] + 0xFFFF; This is better practice computationally, but it corrupts data taking by altering the Mv stored
		if ((Regs.s32[RegPeltierPidMv]+0xFFFF) < pwmCcrMaxHeat) {				
			pwmCcr = pwmCcrMaxHeat;							//Maximum allowed heating
		}
		else {
			pwmCcr = (uint16_t) (Regs.s32[RegPeltierPidMv] + 0xFFFF);		//Partial heating
		}
	}
	TIM3->CCR1 = pwmCcr; 	//SetReg(RegPeltierPidManipulatedVariable, (uint16_t)(manipulatedVariable));		
}
```

This function is called periodically within a while(1) loop. The period of the function call is set by tempBoxcarPeriod. I use boxcar averaging to store several ADC temperature values sampled at regular intervals. Once the boxcar array has been repopulated exactly once with all new values, the controller recalculates MV. Boxcar averaging is required because of poor grounding between the ADC and the PWM used to drive the Peltier device. When the clock changes digital states, the ADC is pulled up or down a couple dozen nano seconds. If the ADC is sampled at this time, and erroneous temperature will be used by the controller. These irregular spikes can be seen in the data for my tuning of $K_p$. Averaging $n$ samples improves the signal-to-noise ratio by $\sqrt{n}$. After finding the standard deviations of the noise and signal, I found $n=9$ almost entirely eliminated the noise.

Also in the while(1) loop, Parse is a function defined in fw\src\commands.c that communicates over USB, to facilitate data taking by my c++ gui. The gui has check boxes for which register's data should be read and at what frequency. The data can be simultaneously viewed in a real time graph, and also exported to a .csv file. To make the graphs in this project, I imported the .csv files into python and plotted them. 

The other important files are regs.c and hwConfig.c. hwConfig.c simply contains all the hardware configuration functions for the peripherals used such as ADC and PWM. Most of the code here was generated by CubeMX and manually edited where required. regs.c stores relevant data that I would want to read or write to via the gui. The functions ReadReg and SetReg also allow the option to trigger additional processes when a register is  read or written to. 

## Known Bugs and Future Features

1. ~~Currently Parse can only send integers, so my GUI can only view data for ADC, and not the conversion to Celsius which requires floats.~~ Implemented
2. Install an interlock on the H-bridge chip. Currently I'm throttling the supplied current to 75% to ensure it never gets too hot--implemented by int pwmCcrMaxCool inside PidControlCalculation(). But with an interlock I could provide higher current for short durations in order to reach colder temperatures in addition to being safer for the hardware. 
3. Meaningfully use the insulated box and additional thermister to create a nested PID loop, with a low frequency outer loop dependant on the inner loop output and a high frequency inner loop sensing the interior of the box temperature. Partially implemented, needs testing and refinement.
