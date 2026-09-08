#!/bin/bash

# Script để build và chạy motion_simulation_2d.cpp
# Build and run script for motion_simulation_2d.cpp

echo "🎯 BUILD AND RUN MOTION SIMULATION 2D"
echo "======================================"

# Kiểm tra compiler
if ! command -v g++ &> /dev/null; then
    echo "❌ Không tìm thấy g++ compiler!"
    echo "Hãy cài đặt g++ trước khi chạy script này."
    exit 1
fi

# Tạo thư mục build nếu chưa có
if [ ! -d "build" ]; then
    echo "📁 Tạo thư mục build..."
    mkdir build
fi

cd build

# Biên dịch
echo "🔨 Đang biên dịch motion_simulation_2d.cpp..."
g++ -std=c++11 -O2 -Wall -Wextra -o motion_simulation_2d ../src/motion_simulation_2d.cpp -lm

if [ $? -eq 0 ]; then
    echo "✅ Biên dịch thành công!"
else
    echo "❌ Lỗi biên dịch!"
    exit 1
fi

# Chạy mô phỏng
echo ""
echo "🚀 Đang chạy mô phỏng..."
./motion_simulation_2d

if [ $? -eq 0 ]; then
    echo "✅ Mô phỏng hoàn thành!"
else
    echo "❌ Lỗi chạy mô phỏng!"
    exit 1
fi

# Kiểm tra file output
if [ -f "trajectory_data.csv" ] && [ -f "proxemic_cost_data.csv" ]; then
    echo "✅ Đã tạo file output thành công!"
    echo "   - trajectory_data.csv"
    echo "   - proxemic_cost_data.csv"
else
    echo "❌ Không tìm thấy file output!"
    exit 1
fi

# Chạy visualization nếu có Python
echo ""
echo "🔄 Kiểm tra Python và dependencies..."

if command -v python3 &> /dev/null; then
    echo "✅ Tìm thấy Python3"
    
    # Kiểm tra numpy
    if python3 -c "import numpy" &> /dev/null; then
        echo "✅ Numpy đã cài đặt"
    else
        echo "⚠️  Numpy chưa cài đặt. Cài đặt với: pip install numpy"
    fi
    
    # Kiểm tra matplotlib
    if python3 -c "import matplotlib" &> /dev/null; then
        echo "✅ Matplotlib đã cài đặt"
    else
        echo "⚠️  Matplotlib chưa cài đặt. Cài đặt với: pip install matplotlib"
    fi
    
    # Kiểm tra pandas
    if python3 -c "import pandas" &> /dev/null; then
        echo "✅ Pandas đã cài đặt"
    else
        echo "⚠️  Pandas chưa cài đặt. Cài đặt với: pip install pandas"
    fi
    
    # Chạy visualization
    echo ""
    echo "🎨 Đang chạy visualization..."
    python3 ../scripts/visualize_motion_simulation.py
    
    if [ $? -eq 0 ]; then
        echo "✅ Visualization hoàn thành!"
        if [ -f "motion_simulation_results.png" ]; then
            echo "   - motion_simulation_results.png"
        fi
    else
        echo "❌ Lỗi visualization!"
    fi
else
    echo "⚠️  Không tìm thấy Python3. Bỏ qua visualization."
    echo "   Cài đặt Python3 để chạy visualization."
fi

echo ""
echo "🎉 HOÀN THÀNH!"
echo "================"
echo "📊 Kết quả mô phỏng đã được lưu trong thư mục build/"
echo "📈 Để xem visualization, chạy: python3 ../scripts/visualize_motion_simulation.py"
echo "📖 Để biết thêm chi tiết, xem README_motion_simulation.md" 