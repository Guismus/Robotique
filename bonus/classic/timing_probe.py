import time
from simulation_test import Simulation

s = Simulation(3, vision=False)
a = time.perf_counter()
for _ in range(1000):
    s.step()
print("1000 pas", time.perf_counter() - a, flush=True)
a = time.perf_counter()
for _ in range(5):
    s.robot.render_camera()
print("5 images", time.perf_counter() - a, flush=True)
s.robot.disconnect()
