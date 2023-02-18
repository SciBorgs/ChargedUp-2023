from casadi import *

nx = 2
nz = 2

x = SX.sym("x", nx)
z = SX.sym("z", nz)
g0 = sin(x + z)
g1 = cos(x - z)
g = Function('g', [x, z], [g0, g1])
G = rootfinder('G', 'newton', g)
print(G)

# angery