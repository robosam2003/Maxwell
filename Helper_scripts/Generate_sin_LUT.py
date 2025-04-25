import numpy as np

# Generate quarter wave sin table
t = np.linspace(0, np.pi/2, 512)

a = np.sin(t)
text = "{"
for i, n in enumerate(a):
    text += f"{n:.16f}, "
    if i % 8 == 7:
        text += "\n"
text += "}"

print(text)
