#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from datetime import datetime
import argparse

class MoveBaseLogAnalyzer:
    def __init__(self, log_file=None):
        self.log_file = log_file
        self.data = None
        
    def load_data(self, log_file=None):
        """Load dữ liệu từ file CSV"""
        if log_file:
            self.log_file = log_file
        elif not self.log_file:
            # Tìm file log mới nhất
            log_dir = os.path.expanduser("~/move_base_logs")
            if os.path.exists(log_dir):
                log_files = glob.glob(os.path.join(log_dir, "move_base_state_*.csv"))
                if log_files:
                    self.log_file = max(log_files, key=os.path.getctime)
                else:
                    raise FileNotFoundError("Không tìm thấy file log nào")
            else:
                raise FileNotFoundError(f"Thư mục log không tồn tại: {log_dir}")
        
        print(f"Loading data from: {self.log_file}")
        self.data = pd.read_csv(self.log_file)
        
        # Chuyển đổi timestamp
        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'])
        
        # Tính thời gian tương đối từ giây đầu tiên
        self.data['Time_Relative'] = (self.data['Timestamp'] - self.data['Timestamp'].iloc[0]).dt.total_seconds()
        
        print(f"Loaded {len(self.data)} data points")
        print(f"Time range: {self.data['Timestamp'].min()} to {self.data['Timestamp'].max()}")
        
    def analyze_state_transitions(self):
        """Phân tích các chuyển đổi trạng thái"""
        if self.data is None:
            print("Vui lòng load data trước")
            return
            
        print("\n=== PHÂN TÍCH CHUYỂN ĐỔI TRẠNG THÁI ===")
        
        # Đếm số lần xuất hiện mỗi trạng thái
        state_counts = self.data['State'].value_counts()
        print("\nSố lần xuất hiện mỗi trạng thái:")
        for state, count in state_counts.items():
            print(f"  {state}: {count} lần")
        
        # Tính thời gian trung bình ở mỗi trạng thái
        state_durations = []
        current_state = None
        start_time = None
        
        for idx, row in self.data.iterrows():
            if row['State'] != current_state:
                if current_state is not None and start_time is not None:
                    duration = row['Time_Relative'] - start_time
                    state_durations.append({
                        'State': current_state,
                        'Duration': duration,
                        'Start_Time': start_time,
                        'End_Time': row['Time_Relative']
                    })
                current_state = row['State']
                start_time = row['Time_Relative']
        
        # Thêm trạng thái cuối cùng
        if current_state is not None and start_time is not None:
            duration = self.data['Time_Relative'].iloc[-1] - start_time
            state_durations.append({
                'State': current_state,
                'Duration': duration,
                'Start_Time': start_time,
                'End_Time': self.data['Time_Relative'].iloc[-1]
            })
        
        durations_df = pd.DataFrame(state_durations)
        if not durations_df.empty:
            print("\nThời gian trung bình ở mỗi trạng thái:")
            avg_durations = durations_df.groupby('State')['Duration'].agg(['mean', 'min', 'max', 'count'])
            for state in avg_durations.index:
                mean_dur = avg_durations.loc[state, 'mean']
                min_dur = avg_durations.loc[state, 'min']
                max_dur = avg_durations.loc[state, 'max']
                count = avg_durations.loc[state, 'count']
                print(f"  {state}: {mean_dur:.2f}s (min: {min_dur:.2f}s, max: {max_dur:.2f}s, count: {count})")
        
        return durations_df
    
    def plot_state_timeline(self, save_plot=True):
        """Vẽ timeline của các trạng thái"""
        if self.data is None:
            print("Vui lòng load data trước")
            return
            
        plt.figure(figsize=(15, 8))
        
        # Tạo mapping từ state sang số
        unique_states = self.data['State'].unique()
        state_mapping = {state: i for i, state in enumerate(unique_states)}
        
        # Vẽ timeline
        state_numbers = [state_mapping[state] for state in self.data['State']]
        plt.plot(self.data['Time_Relative'], state_numbers, 'b-', linewidth=2, marker='o', markersize=4)
        
        # Cấu hình plot
        plt.yticks(range(len(unique_states)), unique_states)
        plt.xlabel('Thời gian (giây)')
        plt.ylabel('Trạng thái')
        plt.title('Timeline Trạng Thái Move Base')
        plt.grid(True, alpha=0.3)
        
        # Thêm annotation cho các chuyển đổi trạng thái
        for i in range(1, len(self.data)):
            if self.data.iloc[i]['State'] != self.data.iloc[i-1]['State']:
                plt.annotate(f"{self.data.iloc[i]['State']}", 
                           xy=(self.data.iloc[i]['Time_Relative'], state_mapping[self.data.iloc[i]['State']]),
                           xytext=(5, 5), textcoords='offset points',
                           fontsize=8, alpha=0.7)
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_file = f"move_base_timeline_{timestamp}.png"
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"Timeline plot đã được lưu: {plot_file}")
        
        plt.show()
    
    def plot_state_distribution(self, save_plot=True):
        """Vẽ phân phối thời gian của các trạng thái"""
        if self.data is None:
            print("Vui lòng load data trước")
            return
            
        durations_df = self.analyze_state_transitions()
        if durations_df.empty:
            return
            
        plt.figure(figsize=(12, 6))
        
        # Tính tổng thời gian cho mỗi trạng thái
        total_durations = durations_df.groupby('State')['Duration'].sum()
        
        # Vẽ bar chart
        states = total_durations.index
        durations = total_durations.values
        
        bars = plt.bar(range(len(states)), durations, alpha=0.7)
        plt.xticks(range(len(states)), states, rotation=45)
        plt.ylabel('Tổng thời gian (giây)')
        plt.title('Phân Phối Thời Gian Các Trạng Thái')
        plt.grid(True, alpha=0.3)
        
        # Thêm giá trị trên bars
        for bar, duration in zip(bars, durations):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{duration:.1f}s', ha='center', va='bottom')
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_file = f"move_base_distribution_{timestamp}.png"
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"Distribution plot đã được lưu: {plot_file}")
        
        plt.show()
    
    def plot_robot_trajectory(self, save_plot=True):
        """Vẽ quỹ đạo robot"""
        if self.data is None:
            print("Vui lòng load data trước")
            return
            
        plt.figure(figsize=(12, 8))
        
        # Vẽ quỹ đạo
        plt.plot(self.data['Robot_X'], self.data['Robot_Y'], 'b-', linewidth=2, alpha=0.7, label='Robot Path')
        
        # Vẽ điểm bắt đầu và kết thúc
        plt.plot(self.data['Robot_X'].iloc[0], self.data['Robot_Y'].iloc[0], 'go', markersize=10, label='Start')
        plt.plot(self.data['Robot_X'].iloc[-1], self.data['Robot_Y'].iloc[-1], 'ro', markersize=10, label='End')
        
        # Vẽ goal points
        if 'Goal_X' in self.data.columns and 'Goal_Y' in self.data.columns:
            goal_points = self.data[['Goal_X', 'Goal_Y']].drop_duplicates()
            plt.plot(goal_points['Goal_X'], goal_points['Goal_Y'], 'm*', markersize=15, label='Goals')
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Quỹ Đạo Robot')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_file = f"robot_trajectory_{timestamp}.png"
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"Trajectory plot đã được lưu: {plot_file}")
        
        plt.show()
    
    def generate_report(self, output_file=None):
        """Tạo báo cáo tổng hợp"""
        if self.data is None:
            print("Vui lòng load data trước")
            return
            
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"move_base_report_{timestamp}.txt"
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("=== BÁO CÁO PHÂN TÍCH MOVE BASE STATE ===\n")
            f.write(f"File log: {self.log_file}\n")
            f.write(f"Thời gian phân tích: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write(f"Tổng số điểm dữ liệu: {len(self.data)}\n")
            f.write(f"Thời gian bắt đầu: {self.data['Timestamp'].min()}\n")
            f.write(f"Thời gian kết thúc: {self.data['Timestamp'].max()}\n")
            f.write(f"Tổng thời gian: {(self.data['Timestamp'].max() - self.data['Timestamp'].min()).total_seconds():.2f} giây\n\n")
            
            # Thống kê trạng thái
            f.write("=== THỐNG KÊ TRẠNG THÁI ===\n")
            state_counts = self.data['State'].value_counts()
            for state, count in state_counts.items():
                percentage = (count / len(self.data)) * 100
                f.write(f"{state}: {count} lần ({percentage:.1f}%)\n")
            
            f.write("\n")
            
            # Phân tích chuyển đổi
            durations_df = self.analyze_state_transitions()
            if not durations_df.empty:
                f.write("=== PHÂN TÍCH CHUYỂN ĐỔI TRẠNG THÁI ===\n")
                avg_durations = durations_df.groupby('State')['Duration'].agg(['mean', 'min', 'max', 'count'])
                for state in avg_durations.index:
                    mean_dur = avg_durations.loc[state, 'mean']
                    min_dur = avg_durations.loc[state, 'min']
                    max_dur = avg_durations.loc[state, 'max']
                    count = avg_durations.loc[state, 'count']
                    f.write(f"{state}: trung bình {mean_dur:.2f}s (min: {min_dur:.2f}s, max: {max_dur:.2f}s, {count} lần)\n")
        
        print(f"Báo cáo đã được lưu: {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Phân tích log move_base state')
    parser.add_argument('--log_file', type=str, help='Đường dẫn đến file log CSV')
    parser.add_argument('--timeline', action='store_true', help='Vẽ timeline')
    parser.add_argument('--distribution', action='store_true', help='Vẽ phân phối')
    parser.add_argument('--trajectory', action='store_true', help='Vẽ quỹ đạo robot')
    parser.add_argument('--report', action='store_true', help='Tạo báo cáo')
    parser.add_argument('--all', action='store_true', help='Thực hiện tất cả phân tích')
    
    args = parser.parse_args()
    
    analyzer = MoveBaseLogAnalyzer(args.log_file)
    
    try:
        analyzer.load_data()
        
        if args.all or not any([args.timeline, args.distribution, args.trajectory, args.report]):
            # Mặc định thực hiện tất cả
            analyzer.analyze_state_transitions()
            analyzer.plot_state_timeline()
            analyzer.plot_state_distribution()
            analyzer.plot_robot_trajectory()
            analyzer.generate_report()
        else:
            if args.timeline:
                analyzer.plot_state_timeline()
            if args.distribution:
                analyzer.plot_state_distribution()
            if args.trajectory:
                analyzer.plot_robot_trajectory()
            if args.report:
                analyzer.generate_report()
                
    except Exception as e:
        print(f"Lỗi: {e}")

if __name__ == '__main__':
    main() 