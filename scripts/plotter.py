import numpy as np
import yaml
import matplotlib.pyplot as plt
import seaborn as sns
import sys

sns.set_style('ticks')
plt.ion()

worldfile = open('/home/arihant/ws/cpp_ws/src/rrt/config/rrt_config.yaml', 'r')
rrtoutfile = open('/home/arihant/ws/cpp_ws/src/rrt/config/rrt_output.yaml', 'r')

worldconfig = yaml.load(worldfile, Loader=yaml.SafeLoader)
rrtout = yaml.load(rrtoutfile, Loader=yaml.SafeLoader)

try:
    xlim = worldconfig['grid']['xlimits']
    ylim = worldconfig['grid']['ylimits']
    startpose = worldconfig['start_pose']
    goalpose = worldconfig['goal_pose']
    obstacles = worldconfig['obstacles']
    goalthreshold = worldconfig['rrt']['goal_closeness_threshold']
    path = rrtout['path']
    tree = rrtout['tree']
except Exception as e:
    print(e)

xrng = (xlim[1] - xlim[0]) * 0.5 * 1.1
xc = (xlim[0] + xlim[1]) * 0.5
xlim = [xc - xrng, xc + xrng]

yrng = (ylim[1] - ylim[0]) * 0.5 * 1.1
yc = (ylim[0] + ylim[1]) * 0.5
ylim = [yc - yrng, yc + yrng]

fig, ax = plt.subplots(figsize=(9, 9))

sns.scatterplot(x=[startpose[0]], y=[startpose[1]], label='start pose', color='g', marker='o', ax=ax, s=100.0)
sns.scatterplot(x=[goalpose[0]], y=[goalpose[1]], label='goal pose', color='m', marker='o', ax=ax, s=100.0)
circle = plt.Circle(goalpose, goalthreshold, color='m', alpha=0.3, label='Goal Region')
ax.add_patch(circle)

ax.set(xlim=xlim, ylim=ylim, xlabel='x', ylabel='y', title='RRT: Searching for path to goal pose', aspect=1.0)
fig.canvas.draw()
fig.canvas.flush_events()

for obs in obstacles:
    if obs['type'] == 'Line':
        x = [obs['start'][0], obs['end'][0]]
        y = [obs['start'][1], obs['end'][1]]
        sns.lineplot(x=x, y=y, color='r', linewidth=2.0)
    if obs['type'] == 'Circle':
        circle = plt.Circle(obs['center'], obs['radius'], color='r')
        ax.add_patch(circle)
    else:
        pass

fig.canvas.draw()
fig.canvas.flush_events()

node_ids = []
nodes = {}
for node in tree:
    node_ids.append(node['id'])
    nodes[node['id']] = node

for node_id in sorted(node_ids):
    try:
        node = nodes[node_id]
        start = node['parent']
        end = node['data']
        sns.lineplot(x=[start[0], end[0]], y=[start[1], end[1]], color='k', marker='o', linewidth=0.5, alpha=0.4, sort=False, ax=ax)
    except Exception as e:
        print(node['id'], ' ', e)

    # if node_id % 20 == 0:
    fig.canvas.draw()
    fig.canvas.flush_events()

fig.canvas.draw()
fig.canvas.flush_events()

ax.set(title='RRT: Path found!')

x = []
y = []
for p in path:
    x.append(p[0])
    y.append(p[1])

for i in range(len(x) - 1):
    sns.lineplot(x=[x[i], x[i+1]], y=[y[i], y[i+1]], color='b', linestyle='-', linewidth=3.0,  marker='o', sort=False, ax=ax)
    fig.canvas.draw()
    fig.canvas.flush_events()

plt.ioff()
plt.show()

worldfile.close()
rrtoutfile.close()