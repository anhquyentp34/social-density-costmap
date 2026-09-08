#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script so sánh kết quả motion simulation với MATLAB motion_simulation_2d.m
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys

def read_matlab_results():
    """Đọc kết quả từ MATLAB (nếu có)"""
    try:
        # Đọc trajectory data từ MATLAB
        if os.path.exists("trajectory_data.csv"):
            trajectory_data = pd.read_csv("trajectory_data.csv", comment='#', delimiter=' ', 
                                       names=['time', 'x', 'y', 'theta', 'vx', 'vy'])
            print("✅ Đọc được trajectory data từ MATLAB")
        else:
            print("❌ Không tìm thấy trajectory_data.csv từ MATLAB")
            trajectory_data = None
            
        # Đọc proxemic cost data từ MATLAB
        if os.path.exists("proxemic_cost_data.csv"):
            # Đọc thông tin lưới từ header
            grid_info = {}
            with open("proxemic_cost_data.csv", 'r') as f:
                for line in f:
                    if line.startswith('#') and ':' in line:
                        key, value = line[1:].strip().split(':', 1)
                        grid_info[key.strip()] = float(value.strip())
            
            # Đọc dữ liệu
            data = pd.read_csv("proxemic_cost_data.csv", comment='#', delimiter=' ', 
                             names=['i', 'j', 'x', 'y', 'cost'])
            
            # Tạo ma trận 2D từ dữ liệu
            nx = int(grid_info['nx'])
            ny = int(grid_info['ny'])
            cost_matrix = np.zeros((ny, nx))
            
            for _, row in data.iterrows():
                i, j = int(row['i']), int(row['j'])
                cost_matrix[i, j] = row['cost']
            
            print("✅ Đọc được proxemic cost data từ MATLAB")
            return trajectory_data, cost_matrix, grid_info
        else:
            print("❌ Không tìm thấy proxemic_cost_data.csv từ MATLAB")
            return trajectory_data, None, None
            
    except Exception as e:
        print(f"❌ Lỗi đọc dữ liệu MATLAB: {e}")
        return None, None, None

def read_cpp_results():
    """Đọc kết quả từ C++ (nếu có)"""
    try:
        # Đọc trajectory data từ C++
        if os.path.exists("cpp_trajectory_data.csv"):
            trajectory_data = pd.read_csv("cpp_trajectory_data.csv", comment='#', delimiter=' ', 
                                       names=['time', 'x', 'y', 'theta', 'vx', 'vy'])
            print("✅ Đọc được trajectory data từ C++")
        else:
            print("❌ Không tìm thấy cpp_trajectory_data.csv từ C++")
            trajectory_data = None
            
        # Đọc proxemic cost data từ C++
        if os.path.exists("cpp_proxemic_cost_data.csv"):
            # Đọc thông tin lưới từ header
            grid_info = {}
            with open("cpp_proxemic_cost_data.csv", 'r') as f:
                for line in f:
                    if line.startswith('#') and ':' in line:
                        key, value = line[1:].strip().split(':', 1)
                        grid_info[key.strip()] = float(value.strip())
            
            # Đọc dữ liệu
            data = pd.read_csv("cpp_proxemic_cost_data.csv", comment='#', delimiter=' ', 
                             names=['i', 'j', 'x', 'y', 'cost'])
            
            # Tạo ma trận 2D từ dữ liệu
            nx = int(grid_info['nx'])
            ny = int(grid_info['ny'])
            cost_matrix = np.zeros((ny, nx))
            
            for _, row in data.iterrows():
                i, j = int(row['i']), int(row['j'])
                cost_matrix[i, j] = row['cost']
            
            print("✅ Đọc được proxemic cost data từ C++")
            return trajectory_data, cost_matrix, grid_info
        else:
            print("❌ Không tìm thấy cpp_proxemic_cost_data.csv từ C++")
            return trajectory_data, None, None
            
    except Exception as e:
        print(f"❌ Lỗi đọc dữ liệu C++: {e}")
        return None, None, None

def compare_results(matlab_cost, cpp_cost, matlab_grid, cpp_grid):
    """So sánh kết quả MATLAB và C++"""
    if matlab_cost is None or cpp_cost is None:
        print("❌ Không thể so sánh do thiếu dữ liệu")
        return
    
    print("\n🔍 SO SÁNH KẾT QUẢ MATLAB vs C++")
    print("=" * 50)
    
    # So sánh kích thước
    print(f"📊 Kích thước MATLAB: {matlab_cost.shape}")
    print(f"📊 Kích thước C++: {cpp_cost.shape}")
    
    # So sánh thống kê
    print(f"\n💰 THỐNG KÊ MATLAB:")
    print(f"   Chi phí tối đa: {np.max(matlab_cost):.2f}")
    print(f"   Chi phí trung bình: {np.mean(matlab_cost):.2f}")
    print(f"   Số điểm có chi phí > 0: {np.sum(matlab_cost > 0)}")
    
    print(f"\n💰 THỐNG KÊ C++:")
    print(f"   Chi phí tối đa: {np.max(cpp_cost):.2f}")
    print(f"   Chi phí trung bình: {np.mean(cpp_cost):.2f}")
    print(f"   Số điểm có chi phí > 0: {np.sum(cpp_cost > 0)}")
    
    # Tính độ khác biệt
    if matlab_cost.shape == cpp_cost.shape:
        diff = np.abs(matlab_cost - cpp_cost)
        max_diff = np.max(diff)
        mean_diff = np.mean(diff)
        print(f"\n📈 ĐỘ KHÁC BIỆT:")
        print(f"   Độ khác biệt tối đa: {max_diff:.6f}")
        print(f"   Độ khác biệt trung bình: {mean_diff:.6f}")
        
        if max_diff < 1e-6:
            print("✅ Kết quả giống hệt nhau!")
        elif max_diff < 1e-3:
            print("✅ Kết quả gần giống nhau (sai số nhỏ)")
        else:
            print("⚠️  Có sự khác biệt đáng kể")
    else:
        print("❌ Không thể so sánh do kích thước khác nhau")

def create_comparison_plot(matlab_cost, cpp_cost, matlab_traj, cpp_traj):
    """Tạo biểu đồ so sánh"""
    if matlab_cost is None or cpp_cost is None:
        print("❌ Không thể tạo biểu đồ do thiếu dữ liệu")
        return
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    # MATLAB results
    axes[0, 0].contourf(matlab_cost, levels=25, cmap='jet')
    axes[0, 0].set_title('MATLAB - Proxemic Cost')
    axes[0, 0].set_aspect('equal')
    
    if matlab_traj is not None:
        axes[0, 1].plot(matlab_traj['x'], matlab_traj['y'], 'w-', linewidth=2)
        axes[0, 1].set_title('MATLAB - Trajectory')
        axes[0, 1].set_aspect('equal')
    
    # C++ results
    axes[1, 0].contourf(cpp_cost, levels=25, cmap='jet')
    axes[1, 0].set_title('C++ - Proxemic Cost')
    axes[1, 0].set_aspect('equal')
    
    if cpp_traj is not None:
        axes[1, 1].plot(cpp_traj['x'], cpp_traj['y'], 'w-', linewidth=2)
        axes[1, 1].set_title('C++ - Trajectory')
        axes[1, 1].set_aspect('equal')
    
    # Difference
    if matlab_cost.shape == cpp_cost.shape:
        diff = np.abs(matlab_cost - cpp_cost)
        im = axes[0, 2].contourf(diff, levels=25, cmap='hot')
        axes[0, 2].set_title('Absolute Difference')
        axes[0, 2].set_aspect('equal')
        plt.colorbar(im, ax=axes[0, 2])
    
    # Histogram of differences
    if matlab_cost.shape == cpp_cost.shape:
        diff_flat = np.abs(matlab_cost.flatten() - cpp_cost.flatten())
        axes[1, 2].hist(diff_flat, bins=50, alpha=0.7)
        axes[1, 2].set_title('Histogram of Differences')
        axes[1, 2].set_xlabel('Absolute Difference')
        axes[1, 2].set_ylabel('Frequency')
    
    plt.tight_layout()
    plt.savefig('matlab_cpp_comparison.png', dpi=300, bbox_inches='tight')
    print("✅ Đã lưu biểu đồ so sánh: matlab_cpp_comparison.png")

def main():
    """Hàm chính"""
    print("🔍 SO SÁNH KẾT QUẢ MATLAB vs C++")
    print("=" * 50)
    
    # Đọc dữ liệu MATLAB
    print("\n📖 Đọc dữ liệu MATLAB...")
    matlab_traj, matlab_cost, matlab_grid = read_matlab_results()
    
    # Đọc dữ liệu C++
    print("\n📖 Đọc dữ liệu C++...")
    cpp_traj, cpp_cost, cpp_grid = read_cpp_results()
    
    # So sánh kết quả
    if matlab_cost is not None and cpp_cost is not None:
        compare_results(matlab_cost, cpp_cost, matlab_grid, cpp_grid)
        create_comparison_plot(matlab_cost, cpp_cost, matlab_traj, cpp_traj)
    else:
        print("\n❌ Không đủ dữ liệu để so sánh")
        print("Hãy chạy MATLAB motion_simulation_2d.m và C++ layer tương ứng trước")

if __name__ == "__main__":
    main() 