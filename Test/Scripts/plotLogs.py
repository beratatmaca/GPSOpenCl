import matplotlib.pyplot as plt

nums = []
with open('../../log.txt') as f:
    lines = f.readlines()
    for line in lines:
        line = line.strip()
        nums.append(float(line))
    plt.plot(nums)
    plt.show()
    f.close()