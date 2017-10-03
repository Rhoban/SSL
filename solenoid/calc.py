import math

resistivity = 17*10**(-9)

# AWG diameter (m)
def awg_diameter(n):
    return 0.001 * 0.127 * 92**((36.0-n)/39)

# AWG section (m^2)
def awg_section(n):
    return (awg_diameter(n)**2) * math.pi/4.0

# AWG ohm/meter
def awg_ohm_per_meter(n):
    global resistivity
    return resistitivy*(1.0/awg_section(n))

awg = 24
diameter = 0.012
length = 0.022
turns = 400
spire = 0
turns_per_level = round(length / awg_diameter(awg))
levels = round(turns / turns_per_level)
turns = turns_per_level * levels
outer_diameter = diameter + 2*levels*awg_diameter(awg)

for l in range(0, levels):
    perimeter = math.pi * diameter
    diameter += awg_diameter(awg)*2
    spire += perimeter*turns_per_level

resistance = resistivity * spire/awg_section(awg)

print('~ AWG %d [diam: %f] ~' % (awg, awg_diameter(awg)))
print('Turns per level: %d' % turns_per_level)
print('Levels: %d' % levels)
print('Number of turns: %d' % turns)
print('Spire length: %f' % spire)
print('Outer diameter: %f' % outer_diameter)
print('Resistance: %f' % resistance)
