import matplotlib.pyplot as plt

# Visualize Occupancy Grid
plt.imshow(occupancy_grid, cmap='gray', origin='upper')
plt.title("Occupancy Grid")
plt.colorbar(label="Occupancy Value")
plt.show()
