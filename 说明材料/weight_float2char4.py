
import numpy as np

img = np.arange(0.01, 1.0, 0.01)
n = img.size

min = np.min(img)

img_char = np.zeros((2, n))
img_tmp = min / img * 0.8 + 0.2
img_char[0] = ((1-img_tmp)*255.0).astype(np.uint8)
img_char[1] = ((img_tmp)*255.0).astype(np.uint8)


for i in range(n):
    print("{} {} {} {}".format(img[i], img_tmp[i], img_char[0][i],img_char[1][i]))

import plotly.graph_objects as go

# 假设你的数据存储在名为data的numpy array中
fig = go.Figure()

# 添加第一条曲线
fig.add_trace(go.Scatter(
    x=img,  # 第一列是x坐标
    y=img_tmp,  # 第二列是第一条曲线的y坐标
    mode='lines',
    name='line 1'
))

# 添加第二条曲线
fig.add_trace(go.Scatter(
    x=img,
    y=img_char[0],  # 第三列是第二条曲线的y坐标
    mode='lines',
    name='line 2'
))

# 添加第三条曲线
fig.add_trace(go.Scatter(
    x=img,
    y=img_char[1],  # 第四列是第三条曲线的y坐标
    mode='lines',
    name='line 3'
))

fig.show()