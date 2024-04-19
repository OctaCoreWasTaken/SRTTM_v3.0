# Copyright OctaCore Studios 2024 (c).
# Made by OctaCore. 18.04.2024.

from matplotlib.pyplot import *
from math import *

Fz = 5000 # N
mu = 0.9 # idk
loadSensitivity = 0.001 # only empirical variable
cpl = 15 # cm
cpw = 20 # cm
stiffness = 4000 # N
slope = 100 # i lied (its empirical)


D2R = pi / 180 # Deg to rad
Ca = stiffness * 10 # Calculate an approximation of Cornering stiffness
maxAlpha = atan((Fz * mu) / Ca) / D2R # maximum slip angle before slip
maxDefl = tan(D2R * maxAlpha) * (cpl / 2) # maximum deflection before slip
k = stiffness / maxDefl # tyre spring constant

def f(x,f):
    return sin(atan(x * f)) # Load sensitivity function

# plotting stuff
alphas = []
fys = []
c2s = []

# Sign of a variable
def sgn(x):
    if x == 0:
        return 0
    elif x > 0:
        return 1
    return -1

# Recalculate maxDefl to calculate the posibility of passing the maximum lateral force (Fz * mu).
# check which will happen first? surpassing Fz * mu or reaching max deflection.
maxDefl = min((f(Fz * mu,loadSensitivity) * Fz * mu) / stiffness,maxDefl)
# Calculate max lateral velocity before slip
# We calculate maxI by reversing the linear lateral force formula
maxI = (maxDefl * k) / (Fz * mu)
# maxAlpha = atan(maxDefl / (cpl / 2)) / D2R # Not needed smh


# for plotting
for i in range(-250,250):
    # Lateral velocity of tyre
    alpha = i * 0.01
    # Linear lateral force. Ff = N * vlateral * (tyreStiffness / N)
    Fy = alpha * Fz * mu # Ff = n * v

    # Fy = -Fs => Fy = kx => x = Fy / k. Calculate deflection
    defl = Fy / k
    slipAngle = atan(defl / (cpl / 2)) / D2R
    
    # Apply loadSensitivity to lateral force. LsFy * LsN (Ls = Load sensitivity)
    Fy = f(Fy,loadSensitivity) * (f(Fz * mu,loadSensitivity) * Fz * mu)
    
    # check if tyre is slipping. if we surpass the maximum allowed deflection or we surpass the maximum lateral force
    if abs(defl) > maxDefl:
        # Set deflection to maximum deflection
        defl = maxDefl * sgn(defl)
        # recalculate Fy. tyreSpringStiffness * deflection (Fy) * N - slope - (abs(currentAlpha) - alphaWhenSlipStart) * sign(currentAlpha)
        Fy = f(k * defl,loadSensitivity) * (f(Fz * mu,loadSensitivity) * Fz * mu) - slope * (abs(alpha) - maxI) * sgn(alpha)
    
    fys.append(Fy)
    # c2s.append(defl * (f(F    z * mu,loadSensitivity) * Fz * mu))
    alphas.append(alpha) # can be changed to slipAngle to get lateral force vs slip angle. By default its set to lateral velocity vs lateral force

     
plot(alphas,fys)
# plot(alphas,c2s)
# plot(alphas,fys)
ylabel("(N) - Lateral Force")
xlabel("(m/s) - Input Lateral Velocity (mu = " + str(mu) + ", Fz = " + str(Fz / 1000) + "kN, k = " + str(int(k / 1000 * 100) / 100) + "kN/cm, Cp_length = " + str(cpl) + "cm)")
grid()
show()
