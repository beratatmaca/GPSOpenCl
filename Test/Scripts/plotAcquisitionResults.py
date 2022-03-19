import matplotlib.pyplot as plt
import numpy as np

nums = []
with open('../../log.txt') as f:
    lines = f.readlines()
    for line in lines:
        line = line.strip()
        nums.append(float(line))
    f.close()
    nums = np.array(nums)

    x = np.arange(-7000, 7500, 500)
    y = np.arange(0, 4096, 1)
    X, Y = np.meshgrid(x, y)
    print(X.shape)
    Z = nums.reshape(X.shape)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, Z, cstride=1, rstride=1, cmap='viridis', shade=False)
    plt.show()