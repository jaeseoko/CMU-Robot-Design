import numpy as np

Ts = 23.5/1000                      # Nm (stall torque)
Is = 1.8                            # A  (stall current)
R = 8.4                             # Ohm
V = 12                              # Voltage [V]
noLoadCurr = 70/1000                # A
noLoadSpeed = 7000*2*np.pi/60       # rad / s

Kt = Ts/Is
Ke = (V - R*noLoadCurr)/noLoadSpeed

print("Kt: ",Kt)
print("Ke: ",Ke)

