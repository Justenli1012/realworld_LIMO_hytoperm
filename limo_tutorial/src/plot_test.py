import matplotlib.pyplot as plt
import time

plt.ion()  # Interactive mode on

fig, ax = plt.subplots()
ax.plot([0, 1, 2], [0, 1, 4], marker='o')
ax.set_title("Test Plot")
plt.show()

plt.pause(0.1)  # Give time to update the plot

print("Plot should be visible now")

time.sleep(10)  # Keep window open for 10 seconds

