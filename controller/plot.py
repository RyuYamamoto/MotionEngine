import pandas as pd
import matplotlib.pyplot as plt

plt.style.use('seaborn-darkgrid')

pd = pd.read_csv('build/foot.csv')
ref_zmp_x = list(pd['ref zmp x'])
ref_zmp_y = list(pd['ref zmp y'])

fig = plt.figure(figsize=(10, 10))
plt.plot(ref_zmp_x, ref_zmp_y, label='reference zmp pattern')
plt.legend()
plt.show()
