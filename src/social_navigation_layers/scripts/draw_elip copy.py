import numpy as np
import matplotlib.pyplot as plt

# Nhập tọa độ 2 điểm (đầu của một trục)
x1, y1 = -3, 0   # Điểm P
x2, y2 = 3, 0    # Điểm Q

# Nhập độ dài của trục còn lại (toàn bộ, tức là 2b)
d_other = 4     # 2b = 4  =>  b = 2

# Tính bán trục theo đường nối P-Q (a)
a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2) / 2

# Tính bán trục còn lại (b)
b = d_other / 2

# Tính tọa độ tâm elip
center_x = (x1 + x2) / 2
center_y = (y1 + y2) / 2

# Tính góc xoay phi (góc giữa đường nối P-Q và trục x)
phi = np.arctan2(y2 - y1, x2 - x1)

# Tạo các giá trị góc từ 0 đến 2π để vẽ elip theo tham số
theta = np.linspace(0, 2*np.pi, 1000)

# Phương trình tham số của elip sau khi xoay:
# (x - center_x) = a*cos(theta)*cos(phi) - b*sin(theta)*sin(phi)
# (y - center_y) = a*cos(theta)*sin(phi) + b*sin(theta)*cos(phi)
x = center_x + a * np.cos(theta) * np.cos(phi) - b * np.sin(theta) * np.sin(phi)
y = center_y + a * np.cos(theta) * np.sin(phi) + b * np.sin(theta) * np.cos(phi)

# Vẽ elip và đánh dấu các điểm
plt.figure(figsize=(8,6))
plt.plot(x, y, label="Elip")
plt.scatter([x1, x2], [y1, y2], color="red", zorder=5, label="Hai đầu của trục")
plt.scatter(center_x, center_y, color="green", zorder=5, label="Tâm elip")
plt.xlabel("Trục x")
plt.ylabel("Trục y")
plt.title("Elip với 2 đầu trục và độ dài trục còn lại đã biết")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
