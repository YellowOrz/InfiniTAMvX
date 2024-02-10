import numpy as np

img = np.arange(0.1, 10.1, 0.1)

min = np.min(img)
max = np.max(img)
scale = 1.0 / (max - min)


def interpolate(val, y0, x0, y1, x1):
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0


def base(val):
    if val <= -0.75:
        return 0.0
    elif val <= -0.25:
        return interpolate(val, 0.0, -0.75, 1.0, -0.25)
    elif val <= 0.25:
        return 1.0
    elif val <= 0.75:
        return interpolate(val, 1.0, 0.25, 0.0, 0.75)
    else:
        return 0.0


img_char = np.zeros((3, 100))
img_tmp = img
for i in range(100):
    img_tmp[i] = (img_tmp[i] - min) * scale
    img_char[0][i] = np.uint8(base(img_tmp[i] - 0.5) * 255.0)
    img_char[1][i] = np.uint8(base(img_tmp[i]) * 255.0)
    img_char[2][i] = np.uint8(base(img_tmp[i] + 0.5) * 255.0)

for i in range(100):
    print("{} {} {} {}".format(img[i], img_tmp[i], img_char[0][i],
                               img_char[1][i]))

import plotly.graph_objects as go

# 假设你的数据存储在名为data的numpy array中
fig = go.Figure()

# 添加第一条曲线
fig.add_trace(
    go.Scatter(
        x=img,  # 第一列是x坐标
        y=img_tmp,  # 第二列是第一条曲线的y坐标
        mode='lines',
        name='line 1'))

# 添加第二条曲线
fig.add_trace(
    go.Scatter(
        x=img,
        y=img_char[0],  # 第三列是第二条曲线的y坐标
        mode='lines',
        name='line 2'))

# 添加第三条曲线
fig.add_trace(
    go.Scatter(
        x=img,
        y=img_char[1],  # 第四列是第三条曲线的y坐标
        mode='lines',
        name='line 3'))

# 添加第四条曲线
fig.add_trace(
    go.Scatter(
        x=img,
        y=img_char[2],  # 第四列是第三条曲线的y坐标
        mode='lines',
        name='line 4'))

fig.show()
