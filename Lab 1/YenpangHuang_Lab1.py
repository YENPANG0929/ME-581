# AEM581 lab 1, circuit python implementation of discrete-time controller for analog plant
###################################################
# import libraries
import board
import time
from analogio import AnalogOut, AnalogIn

###################################################
# Experiment Parameters
Tfinal = 10                          # seconds, length of time to run after the initialization
Tinit = 2                           # seconds, length of time to let the plant settle before running control experiment

# discrete controller timing
Ts = 0.05                           # seconds, sample period for control
Ts_ns = Ts*1e9                      # nanosecond version for better precision. precompute so we don't do an unecessary computation in loop
N = int(Tfinal/Ts)                  # integer number of iterations to do after r or d step
N_init = int(Tinit/Ts)              # integer number of iterations before the step inputs

# reference input
Vop = 0.0                           # volts, the operating point. We will consider the small perturbations about this operating point just as you do in the problem sets.
r_stepMag = 1.0                     # volts, reference signal step magnitude
r = Vop                             # volts, reference signal voltage a.k.a "Setpoint". initialize at the operating point and change if there is an input applied later

# disturbance input 
d_stepMag = 0.0                     # volts, the disturbance voltage
d = 0

# Ctrl gains
kd = 0.0                            # derivative gain
kff = 1.0                           # feedforward gain

###################################################
# hardware configuration
# CAUTION: damage to the board can result if the voltages applied to any of the pins are greater than Vcc+0.5 volts, 
# so for this board, we don't want to apply a voltage of 3.3+0.5=3.8 volts to the analog inputs, for example.
Vfs = 3.3                       # volts, the full-scale output voltage of the microcontroller's DAC.

# Analog input
Y_ai = AnalogIn(board.A3)

# Analog output
U_ao = AnalogOut(board.A0)                          # control signal analog output on pin A0

###################################################
# Functions
# Helper to convert analog input to voltage
def getVoltage(pin):
    return (pin.value * 3.3) / 65536

up, ui, ud, u, usat = 0, 0, 0, Vop, 0               # initialize variables for the controller outputs from the proportional, integral, derivative elements.
# Control law functions

def Cd(y, kd, Ts, y_km1):  # add your inputs
    ###
    # fill in this with CD difference equation

    u = kd*(y - y_km1)/Ts

    ###
    return u

# Feedforward
def uff(r, kff):
    return kff*(r)                                  # volts, feed forward control signal

# saturation limiting
def AOsatFcn(u, lower=0, upper=65535):              # specifying default values for lower and upper in case user doesn't supply them as inputs
    return max(min(u, upper), lower)

# map an input between 0.0-Vfs volts to uint 0-2**16 range
def AOscaleFcn(u):
    return int((u/Vfs)*2**16)

###################################################
# Initialize the plant by setting the analog output to a constant value for a period of time before running the experiment
# U_ao.value = int((0/Vfs)*2**16)                   # make the analog output produce 0 volts                              
# time.sleep(Tinit)                               

###################################################
# initialize some variables
t_km1 = -1                                          # timestamp for the last control execution (k minus 1), every Ts seconds. initialized at -1
y_km1 = 0                                           # value for the previous y, updated every Ts seconds in control law, used for derivative controller 
k=0                                                 # sample no., discrete time index increments at Ts intervals

t0 = time.monotonic_ns()                            # identify the starting time

###################################################
# Run the experiment!
while(k < (N + N_init) ):
    # Step the disturbance and reference inputs at specified time by redefining r and d
    if(k == N_init):
        r = r + r_stepMag
        d = d + d_stepMag
    
    t = time.monotonic_ns() - t0                # get timestamp in nanoseconds relative to t0

    # Read analog voltage on D0
    y = getVoltage(Y_ai)     

    # compute error 
    e = r - y
    
    # control
    ud = Cd(y, kd, Ts, y_km1)                                   # derivative controller term
    u = uff(r, kff) + up + ui - ud              # add all the signals from the seperate control elements, proportional, integral, derivative, and feedforward

    # add disturbance input to the controller output. Then scale to 16-bit unsigned int and saturation limit 
    usat = AOscaleFcn(u + d)                    # scale to unsigned 16-bit range (note: the DAC is 12-bit, but circuitpython scales this to the appropriate range when carrying out the analog output)
    usat = AOsatFcn(usat)                       # apply saturation limits
    
    # write the output signal using the DAC
    U_ao.value = int(usat)                      # cast to an int if needed

    # save current samples for the next iteration (difference equations)
    t_km1 = t
    y_km1 = y

    # print output to console, which you can either see in the terminal or redirect (using the redirect operator '>') into a .csv file
    print(k,',', t,',', y,',', r,',', d,',', u, ',', usat)

    k = k + 1                                   # increment  sample index

    # wait until it is time for next control iteration
    time.sleep(Ts)
