from simulation_test import Simulation
from calibrated_planning import plan_cube, TARGETS

s = Simulation(3)
for c in s.detect():
    if c["color"] != 3:
        continue
    try:
        steps, _ = plan_cube(s.kin, c["xyz"], c["yaw"], TARGETS[5][1], s.q)
        print("TRAJET SOMMET CALCULABLE", c, flush=True)
    except ValueError as error:
        print(error, flush=True)
s.robot.disconnect()
