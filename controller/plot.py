import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.transforms import Affine2D
import math

foot_w = 0.1
foot_h = 0.05

def draw_foot(x, y, yaw, ax):
  a1 = [x + foot_h / 2.0, y + foot_w / 2.0]
  a2 = [x + foot_h / 2.0, y - foot_w / 2.0]
  a3 = [x - foot_h / 2.0, y + foot_w / 2.0]
  a4 = [x - foot_h / 2.0, y - foot_w / 2.0]

  ax.plot([a1, a1], [a3, a4], c='b')
  ax.plot([a2, a2], [a3, a4], c='b')
  ax.plot([a1, a2], [a3, a3], c='b')
  ax.plot([a1, a2], [a4, a4], c='b')

def draw_foot_pos(origin, ax):
  center = Affine2D().translate(foot_w / 2, foot_h / 2)
  return patches.Rectangle((0, 0), foot_w, foot_h, transform=center.inverted() + origin,  fill=False)

plt.style.use('seaborn-darkgrid')

df = pd.read_csv('build/ref_zmp.csv')
ref_zmp_x = list(df['ref zmp x'])
ref_zmp_y = list(df['ref zmp y'])

pd_foot = pd.read_csv('build/foot_step.csv')
right_foot_pos_x = list(pd_foot['right_foot_pos_x'])
right_foot_pos_y = list(pd_foot['right_foot_pos_y'])
left_foot_pos_x = list(pd_foot['left_foot_pos_x'])
left_foot_pos_y = list(pd_foot['left_foot_pos_y'])
yaw = list(pd_foot['yaw'])
leg = list(pd_foot['sup_leg'])

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(1,1,1, aspect=1)
ax.plot(ref_zmp_x, ref_zmp_y, label='reference zmp pattern')
a = Affine2D().rotate(0).translate(0.0, -0.08) + ax.transData
pa = draw_foot_pos(a, ax)
ax.add_patch(pa)
for idx in range(len(right_foot_pos_x)):
  r_center = Affine2D().rotate(-1 * yaw[idx]).translate(right_foot_pos_x[idx], right_foot_pos_y[idx]) + ax.transData
  l_center = Affine2D().rotate(-1 * yaw[idx]).translate(left_foot_pos_x[idx], left_foot_pos_y[idx]) + ax.transData
  if idx == 0 and idx == 1:
    p1 = draw_foot_pos(r_center, ax)
    ax.add_patch(p1)
    p2 = draw_foot_pos(l_center, ax)
    ax.add_patch(p2)
  else:
    if leg[idx] == 1:
      p = draw_foot_pos(r_center, ax)
      ax.add_patch(p)
    elif leg[idx] == 0:
      p = draw_foot_pos(l_center, ax)
      ax.add_patch(p)
ax.legend()
plt.show()
