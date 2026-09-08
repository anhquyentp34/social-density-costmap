#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
from std_msgs.msg import String
from datetime import datetime
import threading
import time

class MoveBaseStateVisualizer:
    def __init__(self):
        rospy.init_node('move_base_state_visualizer', anonymous=True)
        
        # Khởi tạo dữ liệu
        self.timestamps = []
        self.states = []
        self.state_codes = {
            "PENDING": 0,
            "ACTIVE": 1,
            "PREEMPTED": 2,
            "SUCCEEDED": 3,
            "ABORTED": 4,
            "REJECTED": 5,
            "PREEMPTING": 6,
            "RECALLING": 7,
            "RECALLED": 8,
            "LOST": 9,
            "UNKNOWN": -1
        }
        
        # Subscriber
        rospy.Subscriber('/move_base_monitor/current_state', String, self.state_callback)
        
        # Khởi tạo matplotlib
        self.setup_plot()
        
        # Timer để update plot
        self.update_timer = rospy.Timer(rospy.Duration(1.0), self.update_plot)
        
        rospy.loginfo("Move Base State Visualizer đã khởi động")
        
    def setup_plot(self):
        """Thiết lập plot"""
        plt.ion()  # Interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('Move Base State Monitor - Real Time', fontsize=16)
        
        # Plot 1: State timeline
        self.ax1.set_title('Move Base State Timeline')
        self.ax1.set_ylabel('State')
        self.ax1.set_ylim(-1, 10)
        self.ax1.set_yticks(list(self.state_codes.values()))
        self.ax1.set_yticklabels(list(self.state_codes.keys()))
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: State duration histogram
        self.ax2.set_title('State Duration Distribution')
        self.ax2.set_xlabel('State')
        self.ax2.set_ylabel('Duration (seconds)')
        self.ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
    def state_callback(self, msg):
        """Callback cho state changes"""
        current_time = time.time()
        state = msg.data
        
        self.timestamps.append(current_time)
        self.states.append(state)
        
        # Giữ chỉ 1000 điểm dữ liệu gần nhất
        if len(self.timestamps) > 1000:
            self.timestamps = self.timestamps[-1000:]
            self.states = self.states[-1000:]
    
    def update_plot(self, event):
        """Update plot với dữ liệu mới"""
        try:
            if len(self.timestamps) < 2:
                return
                
            # Clear plots
            self.ax1.clear()
            self.ax2.clear()
            
            # Plot 1: State timeline
            self.ax1.set_title('Move Base State Timeline')
            self.ax1.set_ylabel('State')
            self.ax1.set_ylim(-1, 10)
            self.ax1.set_yticks(list(self.state_codes.values()))
            self.ax1.set_yticklabels(list(self.state_codes.keys()))
            self.ax1.grid(True, alpha=0.3)
            
            # Chuyển đổi states thành codes
            state_codes = [self.state_codes.get(state, -1) for state in self.states]
            
            # Vẽ timeline
            relative_times = [t - self.timestamps[0] for t in self.timestamps]
            self.ax1.plot(relative_times, state_codes, 'b-', linewidth=2, marker='o', markersize=4)
            
            # Highlight current state
            if self.states:
                current_state = self.states[-1]
                current_code = self.state_codes.get(current_state, -1)
                self.ax1.axhline(y=current_code, color='red', linestyle='--', alpha=0.7, label=f'Current: {current_state}')
                self.ax1.legend()
            
            # Plot 2: State duration histogram
            self.ax2.set_title('State Duration Distribution')
            self.ax2.set_xlabel('State')
            self.ax2.set_ylabel('Duration (seconds)')
            self.ax2.grid(True, alpha=0.3)
            
            # Tính toán duration cho mỗi state
            state_durations = self.calculate_state_durations()
            if state_durations:
                states = list(state_durations.keys())
                durations = list(state_durations.values())
                
                bars = self.ax2.bar(range(len(states)), durations, alpha=0.7)
                self.ax2.set_xticks(range(len(states)))
                self.ax2.set_xticklabels(states, rotation=45)
                
                # Thêm giá trị trên bars
                for bar, duration in zip(bars, durations):
                    height = bar.get_height()
                    self.ax2.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                                f'{duration:.1f}s', ha='center', va='bottom')
            
            plt.tight_layout()
            plt.pause(0.01)
            
        except Exception as e:
            rospy.logerr(f"Lỗi khi update plot: {e}")
    
    def calculate_state_durations(self):
        """Tính toán thời gian duration cho mỗi state"""
        if len(self.timestamps) < 2:
            return {}
            
        state_durations = {}
        current_state = self.states[0]
        start_time = self.timestamps[0]
        
        for i in range(1, len(self.states)):
            if self.states[i] != current_state:
                # State thay đổi, tính duration
                duration = self.timestamps[i-1] - start_time
                if current_state in state_durations:
                    state_durations[current_state] += duration
                else:
                    state_durations[current_state] = duration
                
                # Bắt đầu state mới
                current_state = self.states[i]
                start_time = self.timestamps[i]
        
        # Thêm duration cho state cuối cùng
        if current_state:
            duration = self.timestamps[-1] - start_time
            if current_state in state_durations:
                state_durations[current_state] += duration
            else:
                state_durations[current_state] = duration
        
        return state_durations
    
    def save_plot(self, filename=None):
        """Lưu plot hiện tại"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"move_base_state_plot_{timestamp}.png"
        
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        rospy.loginfo(f"Plot đã được lưu: {filename}")

def main():
    try:
        visualizer = MoveBaseStateVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'visualizer' in locals():
            visualizer.save_plot()

if __name__ == '__main__':
    main() 