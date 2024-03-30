# AEM581 Lab 3, circuit python implementation of discrete-time controller for 4th order analog plant
###################################################
# import libraries
import board
import time
from analogio import AnalogOut, AnalogIn
import digitalio
from digitalio import DigitalInOut, Direction, Pull
# import adafruit_dotstar           # Uncomment this line to visualize the output using the board's LED.


# PID gains
kp = 0.2                           # proportional gain
ki = 0.2                            # integral gain
kd = 0.25                           # derivative gain
kff = 0.92                           # feedforward gain

# discrete controller timing
Ts = 0.050/2                          # seconds, sample period for control
Ts_ns = Ts*1e9                      # nanosecond version for better precision. precompute so we don't do an unecessary computation in loop
Tfinal = 30                          # seconds, length of time to run after the initialization
Tinit = 1                           # seconds, length of time to let the plant settle before enabling the PID controller
N_init = int(Tinit/Ts)              # integer number of iterations for the plant initialization period, before PID controller is enabled
N = int(Tfinal/Ts)                  # integer total number of iterations to do after r or d step

# Reference input
Trstep = 1.0                        # seconds, the time to apply the reference step
r_stepMag = 1.0                     # volts, reference signal step magnitude
Vop = 0.0                           # volts, the operating point. This will be the initial reference value so it will affect any initial transients when the board boots.
N_rstep = int(Trstep/Ts)            # integer number of iterations before the r step input
r = Vop                             # volts, reference signal voltage a.k.a "Setpoint". We initialize at the operating point, and it will change if there is an input applied later

# Disturbance input
Tdstep = 1.0                        # seconds, the time to apply the disturbance step input
d_stepMag = 0.0                     # volts, the disturbance voltage
N_dstep = int(Tdstep/Ts)            # integer number of iterations before the d step input
d = 0                               # disturbance input, initialized at zero and 

###################################################
###################################################
# hardware configuration
# CAUTION: damage to the board can result if the voltages applied to any of the pins are greater than Vcc+0.5 volts, 
# so for this board, we don't want to apply a voltage of 3.3+0.5=3.8 volts to the analog inputs, for example.
Vfs = 3.3                       # volts, the full-scale output voltage of the microcontroller's DAC.

# Analog input
Y_ai = AnalogIn(board.A3)

# Analog outputs
U_ao = AnalogOut(board.A0)                          # control signal analog output on pin A0
D_ao = AnalogOut(board.A1)                          # disturbance signal analog output on pin A1

# Digital IOs
# timingLight = digitalio.DigitalInOut(board.D7)      # pin D7 as a digital output so we can verify the sample timing by oscilloscope
# timingLight.direction = digitalio.Direction.OUTPUT  # set it up as an output.
# uncomment the below line to visualize the output using the board's LED
# led = adafruit_dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1)

###################################################
# Functions
# Helper to convert analog input to voltage
def getVoltage(pin):
    return (pin.value * 3.3) / 65536

# saturation limiting
def AOsatFcn(u, lower=0, upper=65535):              # specifying default values for lower and upper in case user doesn't supply them as inputs
    return max(min(u, upper), lower)

# map an input between 0.0-Vfs volts to uint 0-2**16 range
def AOscaleFcn(u):
    return int((u/Vfs)*2**16)

###################################################
# initialize some variables
up, ui, ud, u, usat = 0, 0, 0, 0, 0               # initialize variables for the controller outputs from the proportional, integral, derivative elements.
t_c, t_d = 0, 0                                     # initialized variables for cont and disc timestamps
y_c, y_d = 0, [0, 0]                                        # value for the previous y, updated every Ts seconds in control law, used for derivative controller 
digout = False                                      # initialize intial value for the output (False = digital low)

k=0                                                 # sample no., discrete time index increments at Ts intervals
t0 = time.monotonic_ns()                            # identify the starting time

T_printns = 1700000                                 # loop timing fudge factor to get the discrete samples closer to Ts. This the approximate time taken to carry out a serial print and the intentional additional delay, compensating for this below will improve our timing accuracy somewhat.

# loop
while (k < (N + N_init)):
    
    t_c = (time.monotonic_ns() - t0)                # get timestamp in nanoseconds relative to t0
    y_c = getVoltage(Y_ai)                      # measure the fast-sampled plant state

    # print(k,',', t_c, ',', y_c, ',', t_d, ',', y_d[1], ',', r,',', d,',', u, ',', usat)
    print(f'{k:d}, {t_c:11d}, {y_c:6.4f}, {t_d:11d}, {y_d[1]:6.4f}, {r:5.3f}, {d:5.3f}, {u:6.4f}, {usat:6.4f}')

    if( t_c - t_d + T_printns >= Ts_ns ):           # It is time for the next iteration of the discrete controller
        # Note that the code below implements the given block diagram of this closed-loop system, 
        # except that the error signal is sampled instead of continuous
        # and the step functions are applied in discrete time at the Ts interval, instead of in continuous time (no problem)
        
        # apply the step functions at the correct time
        if(k == N_rstep):
            r = r + r_stepMag
        if(k == N_dstep):
            d = d + d_stepMag
        
        t_d = t_c                                   # save the current time as the time of this discrete sample

        y_d[0] = y_d[1]                             # save the previous measurement to the k minus 1 position, before acquiring a new measurement
        y_d[1] = y_c                                # Read analog voltage on D0 and save to the kth position in the list

        # compute (sampled) error signal
        e = r - y_d[1]
            
        # control
        if( k >= N_init): 
            #enable control only after the plant initialization period has finished
#################################################################################################
# IMPLEMENT YOUR DIFFERENCE EQUATIONS HERE            
            up = kp*e
            ui = ui + Ts*ki*e
            ud = kd*(y_d[1] - y_d[0])/Ts
#################################################################################################
            uff = kff*(r)                           # feedforward control signal
        else:
            up, ud, ui, uff = 0, 0, 0, 0
        
        
        u = uff + up + ui - ud                      # Sum all the signals from the seperate control elements, proportional, integral, derivative, and feedforward
 
        # add disturbance input to the controller output. Then scale to 16-bit unsigned int and saturation limit 
        usat = AOsatFcn(u, 0, 3.3)                  # apply saturation limits, within the range of possible voltages      
        vout = AOsatFcn(AOscaleFcn(u + d))       # scale to unsigned 16-bit range (note: the DAC is 12-bit, but circuitpython scales this to the appropriate range when carrying out the analog output)
        
        U_ao.value = vout                           # write the output signal using the DAC          

        # Uncomment this line to visualize the output using the board's LED.
        # led[0] = (0, y_c/Vfs * 255, 0)              # write the fast-sampled measurement of the plant state, y, to the LED. 

        k = k + 1                                   # increment  sample index

    time.sleep(0.003)                               # feel free to reduce this if it doesn't cause problems for you.
    # This is a delay before starting next fast sample iteration