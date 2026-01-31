from fmpy import simulate_fmu
import time

start_time = time.time()

# simulate_fmu(
#     './Build/fmu/TestFrDoubleWishboneBase.fmu',
#     start_time=0,
#     stop_time=1,
#     solver='CVode',
#     relative_tolerance=1e-8,
#     fmi_type='ModelExchange',
#     fmi_call_logger=lambda s: print(s),
#     logger=print
# )

# res = simulate_fmu(
#     './Build/fmu/TestFrRigidAxleBellcrank.fmu',
#     start_time=0,
#     stop_time=3,
#     solver='CVode',
#     relative_tolerance=1e-6,
#     fmi_type='ModelExchange',
# )

res = simulate_fmu(
    './Build/fmu/TestFrRigidAxleBellcrank.fmu',
    start_time=0,
    stop_time=6,
    solver='CVode',
    apply_default_start_values=True,
    relative_tolerance=1e-6,
    fmi_type='ModelExchange',
)

end_time = time.time()

print(end_time - start_time)

import matplotlib.pyplot as plt

print(len(res['time']))
# plt.plot(res['time'], res['test'])
# plt.show()
