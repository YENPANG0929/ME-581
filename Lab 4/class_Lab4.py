"""
AEM581 Lab 4, circuit python implementation of discrete-time controller for 5th order analog plant

To make your modifications, look for the edit tags such as:
  ### suggested edits only between this line and the ### below
    ...code you can/should change...
  ###
and edit code between the tags.

"""
###################################################
# import libraries
import board
import time
from analogio import AnalogOut, AnalogIn

# import adafruit_dotstar # uncomment this line and similar in remainder of code for the inbuilt LED visual output.
                                  

### suggested edits only between this line and the ### below
# Discrete control system parameters
Ts = 0.025 *1e9                                 # nanoseconds, sample period for control. This timing is done by polling instead of in hard-real time which is not preferable, so check your sample timing!

# Controller or compensator coeffs
# Note: Cz_num, Cz_den, Nz_num, Nz_den obtained from the design software are listed in descending powers of 'z'
Cz_num = [1, -0.5]                  # Put your coefficients here!    
Cz_den = [1, -0.5]                  

# Notch filter coeffs
Nz_num = [1, -1.949, 0.9724]        # Put your coefficients here!  
Nz_den = [1, -1.949, 0.9724]   

# feedforward control element 
DC_gain = 1.0                       # DCgain of the plant to the inputs (theoretically the same for r-to-y as d-to-y) 
kff = 1.0/(DC_gain)                 # feedforward gain

# operating point of the system. We use 3.3>Vop>0 to avoid nonlinear saturation of the analog inputs/outputs
Vop = 1.5                           # volts, the operating point for the system (same concept as your earlier homeworks)
###

# allocate lists of the correct length for input signals to the (proper or strictly proper) difference equations. 
    # We need to remember N samples of the input signals if N is the order of the difference equation
    # assign initial conditions assuming the plant was at rest at setpoint, i.e., r=Vop
    # using lists instead of preferred numpy arrays just for convenience on the circuit python board. 
    # A note on the commands below: The commands create lists which have the number of elements depending on the length of the corresponding signal, and it assigns the same value to every element of that list. 
    # Ex: if Nz_num = [1, -1.949, 0.9724], then uf = [kff*r, kff*r]. Disambiguation: the '*' operator is NOT multiplying the element '0' of [0] by 2. It is multiplying the LENGTH of the list by 2.
e =  [0] * (len(Cz_num))            # C(k) input. 
uc = [0] * (len(Cz_den) - 1)        # C(k) output signal. IC = 0 since assuming error was zero before t=tstep
uf = [kff*Vop] * (len(Nz_num))      # N(k) input signal.  IC = kff*r ~= Vop since the uff component is initialliy assumed to be Vop=kff*r, while uc component is initially zero
uk = [kff*Vop] * (len(Nz_den) - 1)  # N(k) output signal, IC = kff*r ~= Vop since N(k) is assumed to have been at rest initially.

# hardware configuration
    # CAUTION: damage to the board can result if the voltages applied to any of the pins are greater than Vcc+0.5 volts, 
    # so for this board, we don't want to apply a voltage above 3.3+0.5=3.8 volts to the analog inputs, for example.
Vfs = 3.3                           # volts, the full-scale output voltage of the microcontroller's DAC.

# Analog input
Y_ai = AnalogIn(board.A3)

# Analog outputs
U_ao = AnalogOut(board.A0)                          # control signal analog output on pin A0
D_ao = AnalogOut(board.A1)                          # disturbance signal analog output on pin A1

# uncomment the line below for the inbuilt LED visual output
# led = adafruit_dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1)     

# Functions
# Helper to convert analog input to voltage
def getVoltage(pin):
    return (pin.value * 3.3) / 65536

# saturation limiting
def saturation(u, lower=0, upper=65535):              # specifying default values for lower and upper in case user doesn't supply them as inputs
    return max(min(u, upper), lower)

# map an input between 0.0-Vfs volts to uint 0-2**16 range
def map0to3v_to_uint16(u):
    return int((u/Vfs)*2**16)

def dot(a, b): # dot product of two lists
    out = 0 
    for idx in range(len(a)): 
        out += a[idx] * b[idx]
    return out 

# initialize some variables for the first print output.  
r = Vop                                             # volts, reference signal voltage a.k.a "Setpoint". We initialize at the operating point, and it will change if there is an input applied later
d = 0                                               # volts, disturbance input, initialized at zero  
usat = Vop                                          # volts, the saturation limited controller output u(k).  
t_c, t_d = 0, 0                                     # initialized variables for cont and disc timestamps
y_c, y_d = Vop, Vop                                 # value for the previous y, updated every Ts seconds in control law, used for derivative controller 
k=0                                                 # sample no., discrete time index increments at Ts intervals

# initialize plant, let it settle out at the DCgain near the Vop before the step response experiment
# write the output signals using the DAC
U_ao.value = map0to3v_to_uint16(Vop)                # scale to unsigned 16-bit range (note: the DAC is 12-bit, but circuitpython scales this to the appropriate range when carrying out the analog output)                           # write the output signal using the DAC    
D_ao.value = map0to3v_to_uint16(d)                                  
time.sleep(10.0)                                    # seconds, length of time to let the plant settle before the experiment

# Run experiment!
t0 = time.monotonic_ns()                            # identify the starting time

### suggested edits only between this line and the ### below
while (t_c < 8e9):                                 # nanoseconds, length of time to run the experiment
###
    
    t_c = (time.monotonic_ns() - t0)                # get timestamp in nanoseconds relative to t0
    y_c = getVoltage(Y_ai)                          # measure the fast-sampled plant state

    print(f'{k:d}, {t_c:11d}, {y_c:6.4f}, {t_d:11d}, {y_d:6.4f}, {r:5.3f}, {d:5.3f}, {uk[0]:6.4f}, {usat:6.4f}, {uc[0]:6.4f}')

    if( t_c - t_d  >= Ts-1500000 ):                 # Check if it is time to do the next iteration of the discrete controller with a loop timing fudge factor to get the discrete samples closer to Ts. Fudge factor is the approximate time taken to carry out a serial print.

        # The code below implements the given block diagram of this closed-loop system

        # sample the output  
        t_d = t_c                                   # save the discrete timestamp
        y_d = y_c                                   # Read analog voltage on D0 and save to the kth position in the list
### suggested edits only between this line and the ### below
        # apply the step functions at the right time
        if(t_d > 1.0 *1e9):                         # ref step time (ns)
            r = Vop + 1.0                           # volts, reference signal after step
        if(t_d > 1.0 *1e9):                         # disturbance step time (ns)
            d = 0 + 0.0                             # volts, disturbance signal after step
###        
        # control law
        e = [r - y_d] + e[:-1]                      # error signal
        uff = kff*(r)                               # feedforward control signal

        # 1st order compensator
### suggested edits only between this line and the ### below
        new = 0                                     # implement difference equation here!
###                                   
        uc = [new] + uc[:-1]                        # Python note: the '+' is a list concatenation, not an addition operator
        
        # notch filter input signal
        uf = [uc[0] + uff] + uf[:-1]                

        # 2nd order notch filter. 
### suggested edits only between this line and the ### below
        new = uf[0]                                 # implement difference equation here!
###
        uk = [new] + uk[:-1]

        # Scale analog outputs to 16-bit unsigned int and apply saturation limits 
        usat  = saturation(uk[0], 0, 3.3)                   # apply saturation limits, within the range of possible voltages      
        vuout = saturation( map0to3v_to_uint16(uk[0]) )     # scale to unsigned 16-bit range (note: the DAC is 12-bit, but circuitpython scales this to the appropriate range when carrying out the analog output)
        vdout = saturation( map0to3v_to_uint16(d) )         # scale to unsigned 16-bit range (note: the DAC is 12-bit, but circuitpython scales this to the appropriate range when carrying out the analog output)
        
        # write the output signal(s) using the DAC 
        U_ao.value = vuout                                      
        D_ao.value = vdout
        
        # uncomment the line below for the inbuilt LED visual output
        # led[0] = (0, y_c/Vfs * 255, 0)                    # write the fast-sampled measurement of the plant state, y, to the LED. 

        k = k + 1                                           # increment  sample index
    
    # delay before starting next fast-sample (psuedo-continuous-time) iteration
    time.sleep(0.003)                                       # feel free to reduce this delay to get more samples, if it doesn't cause problems for you.
    