import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import ArtistAnimation
import os

base = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.normpath(os.path.join(base, 'data/data-tmp.json'))

data=pd.read_json(data_path)
uniq=data['points'].drop_duplicates().reset_index(drop=True)

fig = plt.figure(facecolor="w")
# 0 <=x < 2pi の範囲の点列を作成。
ims=[]
for points in uniq:
    im=[]
    for single_route in points:
        tmp=np.array(single_route).T
        im=im+plt.plot(tmp[0],tmp[1],marker='o')
    ims.append(im)
ani = ArtistAnimation(fig, ims, interval=500)
#ani.save('animate.gif')
plt.show()
# mp4ファイルに保存
#ani.save('animation.mp4', writer='ffmpeg')
# gifファイルに保存する場合
# ani.save('animation.gif', writer='pillow')