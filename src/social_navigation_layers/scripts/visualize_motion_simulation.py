#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualization script for motion_simulation_2d.cpp results
Hiển thị kết quả từ mô phỏng C++ với proxemic cost
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
from mpl_toolkits.mplot3d import Axes3D

def read_trajectory_data(filename):
    """Đọc dữ liệu quỹ đạo từ file CSV"""
    try:
        data = pd.read_csv(filename, comment='#', delimiter=' ', 
                          names=['time', 'x', 'y', 'theta', 'vx', 'vy'])
        return data
    except Exception as e:
        print(f"❌ Lỗi đọc file trajectory: {e}")
        return None

def read_proxemic_cost_data(filename):
    """Đọc dữ liệu proxemic cost từ file CSV"""
    try:
        # Đọc thông tin lưới từ header
        grid_info = {}
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#') and ':' in line:
                    key, value = line[1:].strip().split(':', 1)
                    grid_info[key.strip()] = float(value.strip())
        
        # Đọc dữ liệu
        data = pd.read_csv(filename, comment='#', delimiter=' ', 
                          names=['i', 'j', 'x', 'y', 'cost'])
        
        # Tạo ma trận 2D từ dữ liệu
        nx = int(grid_info['nx'])
        ny = int(grid_info['ny'])
        cost_matrix = np.zeros((ny, nx))
        
        for _, row in data.iterrows():
            i, j = int(row['i']), int(row['j'])
            cost_matrix[i, j] = row['cost']
        
        return cost_matrix, grid_info
    except Exception as e:
        print(f"❌ Lỗi đọc file proxemic cost: {e}")
        return None, None

def create_visualization(trajectory_data, cost_matrix, grid_info):
    """Tạo visualization cho kết quả mô phỏng"""
    
    # Tạo figure với 2 subplot
    fig = plt.figure(figsize=(16, 8))
    
    # Subplot 1: 3D Surface
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Tạo lưới cho 3D plot
    x_min, x_max = grid_info['x_min'], grid_info['x_max']
    y_min, y_max = grid_info['y_min'], grid_info['y_max']
    nx, ny = int(grid_info['nx']), int(grid_info['ny'])
    
    X = np.linspace(x_min, x_max, nx)
    Y = np.linspace(y_min, y_max, ny)
    X_grid, Y_grid = np.meshgrid(X, Y)
    
    # Vẽ surface 3D
    surf = ax1.plot_surface(X_grid, Y_grid, cost_matrix, 
                           cmap='jet', alpha=0.8, linewidth=0)
    
    # Vẽ quỹ đạo
    ax1.plot3D(trajectory_data['x'], trajectory_data['y'], 
               np.zeros_like(trajectory_data['x']), 'w-', linewidth=3)
    
    # Vẽ điểm bắt đầu
    ax1.scatter([trajectory_data['x'].iloc[0]], [trajectory_data['y'].iloc[0]], [0], 
                c='red', s=100, marker='o')
    
    ax1.set_title('Proxemic Cost 3D Surface', fontsize=16, fontweight='bold')
    ax1.set_xlabel('X (m)', fontsize=14)
    ax1.set_ylabel('Y (m)', fontsize=14)
    ax1.set_zlabel('Proxemic Cost', fontsize=14)
    ax1.view_init(45, 30)
    
    # Subplot 2: 2D Contour
    ax2 = fig.add_subplot(122)
    
    # Vẽ contour
    contour = ax2.contourf(X_grid, Y_grid, cost_matrix, 25, cmap='jet')
    
    # Vẽ quỹ đạo
    ax2.plot(trajectory_data['x'], trajectory_data['y'], 'w-', linewidth=3)
    
    # Vẽ điểm bắt đầu
    ax2.scatter(trajectory_data['x'].iloc[0], trajectory_data['y'].iloc[0], 
                c='red', s=100, marker='o', zorder=5)
    
    # Vẽ mũi tên hướng di chuyển
    for i in range(0, len(trajectory_data), 10):  # Vẽ mũi tên mỗi 10 điểm
        if i < len(trajectory_data) - 1:
            dx = trajectory_data['x'].iloc[i+1] - trajectory_data['x'].iloc[i]
            dy = trajectory_data['y'].iloc[i+1] - trajectory_data['y'].iloc[i]
            ax2.arrow(trajectory_data['x'].iloc[i], trajectory_data['y'].iloc[i], 
                     dx, dy, head_width=0.1, head_length=0.1, fc='white', ec='white')
    
    ax2.set_title('Proxemic Cost Contour 2D', fontsize=16, fontweight='bold')
    ax2.set_xlabel('X (m)', fontsize=14)
    ax2.set_ylabel('Y (m)', fontsize=14)
    ax2.set_aspect('equal')
    
    # Thêm colorbar
    plt.colorbar(contour, ax=ax2, location='right')
    
    plt.tight_layout()
    return fig

def print_statistics(trajectory_data, cost_matrix):
    """In thống kê kết quả"""
    print("\n📈 THỐNG KÊ KẾT QUẢ:")
    
    # Thống kê quỹ đạo
    total_distance = 0.0
    for i in range(1, len(trajectory_data)):
        dx = trajectory_data['x'].iloc[i] - trajectory_data['x'].iloc[i-1]
        dy = trajectory_data['y'].iloc[i] - trajectory_data['y'].iloc[i-1]
        total_distance += np.sqrt(dx*dx + dy*dy)
    
    total_rotation = trajectory_data['theta'].iloc[-1] - trajectory_data['theta'].iloc[0]
    
    print(f"   Quãng đường: {total_distance:.2f} m")
    print(f"   Góc quay: {total_rotation:.2f} rad ({np.degrees(total_rotation):.1f}°)")
    print(f"   Vị trí cuối: [{trajectory_data['x'].iloc[-1]:.2f}, {trajectory_data['y'].iloc[-1]:.2f}]")
    
    # Thống kê proxemic cost
    max_cost = np.max(cost_matrix)
    mean_cost = np.mean(cost_matrix)
    nonzero_points = np.sum(cost_matrix > 0)
    total_points = cost_matrix.size
    ratio = 100.0 * nonzero_points / total_points
    
    print(f"\n📊 THỐNG KÊ PROXEMIC COST:")
    print(f"   Chi phí tối đa: {max_cost:.2f}")
    print(f"   Chi phí trung bình: {mean_cost:.2f}")
    print(f"   Số điểm có chi phí > 0: {nonzero_points}")
    print(f"   Tỷ lệ điểm có chi phí: {ratio:.1f}%")

def main():
    """Hàm chính"""
    print("🎯 VISUALIZATION CHO MOTION SIMULATION 2D")
    print("==========================================")
    
    # Kiểm tra file đầu vào
    trajectory_file = "trajectory_data.csv"
    cost_file = "proxemic_cost_data.csv"
    
    if not os.path.exists(trajectory_file):
        print(f"❌ Không tìm thấy file: {trajectory_file}")
        print("Hãy chạy motion_simulation_2d.cpp trước!")
        return
    
    if not os.path.exists(cost_file):
        print(f"❌ Không tìm thấy file: {cost_file}")
        print("Hãy chạy motion_simulation_2d.cpp trước!")
        return
    
    # Đọc dữ liệu
    print("🔄 Đang đọc dữ liệu...")
    trajectory_data = read_trajectory_data(trajectory_file)
    cost_matrix, grid_info = read_proxemic_cost_data(cost_file)
    
    if trajectory_data is None or cost_matrix is None:
        print("❌ Lỗi đọc dữ liệu!")
        return
    
    print("✅ Hoàn thành đọc dữ liệu!")
    
    # Tạo visualization
    print("🔄 Đang tạo visualization...")
    fig = create_visualization(trajectory_data, cost_matrix, grid_info)
    
    # In thống kê
    print_statistics(trajectory_data, cost_matrix)
    
    # Hiển thị plot
    plt.show()
    
    # Lưu figure
    output_file = "motion_simulation_results.png"
    fig.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ Đã lưu kết quả vào: {output_file}")

if __name__ == "__main__":
    main() 