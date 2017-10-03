import math

# AWG diameter (cm)
def awg_diameter(n):
    return 0.1 * 0.127 * 92**((36.0-n)/39)

# AWG section (cm^2)
def awg_section(n):
    return (awg_diameter(n)**2) * math.pi/4.0

awg = 24                    # cable type
section = awg_section(awg)  # cm2
mass = 250                  # g
density = 8.93              # g/cm3

length = (mass / (section * density))/100.0
print('AWG [%d] diameter: %f cm' % (awg, awg_diameter(awg)))
print('Length: %.2f m' % length)
