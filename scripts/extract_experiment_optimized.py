#!/usr/bin/env python3

import rosbag
import csv
import sys
import os
import time

def extract_experiment_data_optimized(bag_file, extract_timeline=False, output_dir=None):
    """Trích xuất dữ liệu thí nghiệm với tối ưu hóa hiệu suất"""
    
    # Convert to absolute path if relative
    if not os.path.isabs(bag_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        rosbag_analysis_dir = os.path.dirname(script_dir)
        bag_file = os.path.join(rosbag_analysis_dir, bag_file.lstrip('./').lstrip('../'))
    
    # Extract base name from bag file for output CSV
    bag_basename = os.path.splitext(os.path.basename(bag_file))[0]
    output_csv_name = f"{bag_basename}_extracted_optimized.csv"
    
    print(f"=== CHƯƠNG TRÌNH TRÍCH XUẤT THÍ NGHIỆM (TỐI ƯU) ===")
    print(f"Trích xuất dữ liệu thí nghiệm từ {bag_file}...")
    print(f"Bao gồm: Actors positions, Social metrics (SII, SGI), Move Base Status")
    if extract_timeline:
        print(f"➕ Thêm: Timeline data cho visualization")
    if output_dir:
        print(f"Kết quả: tạo file CSV tổng hợp trong {output_dir}")
    else:
        print("Kết quả: tạo file CSV tổng hợp trong thư mục results (mặc định)")
    
    # Check if bag file exists
    if not os.path.exists(bag_file):
        print(f"❌ Bag file không tồn tại: {bag_file}")
        return False
    
    # Mapping status codes với mô tả chi tiết từ actionlib_msgs
    status_names = {
        0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED", 4: "ABORTED",
        5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING", 8: "RECALLED", 9: "LOST"
    }
    
    start_time = time.time()
    
    try:
        bag = rosbag.Bag(bag_file, allow_unindexed=True)
        
        # Get bag info for progress tracking
        bag_info = bag.get_type_and_topic_info()
        total_messages = sum(bag_info[1][topic].message_count for topic in bag_info[1])
        print(f"📊 Tổng số messages trong bag: {total_messages}")
        
        # Get experiment start time for timeline data
        experiment_start = bag.get_start_time()
        print(f"⏰ Thời gian bắt đầu thí nghiệm: {experiment_start}")
        
        # Initialize data containers
        actors_data = []
        amcl_pose_data = []  # Robot pose trên robot thật (không có /gazebo/model_states)
        sii_data = []
        sgi_data = []
        move_base_status_data = []
        path_planning_timeline = []  # Always initialize for MATLAB compatibility
        all_actors = set()
        
        # Timeline data containers (if enabled)
        if extract_timeline:
            move_base_timeline = []
            robot_trajectory = []
            social_metrics_timeline = []
            actors_trajectory = {}
        
        print(f"\n[1/1] Trích xuất tất cả dữ liệu trong 1 lần quét...")
        
        # Single pass extraction - read all topics at once
        message_count = 0
        for topic, msg, t in bag.read_messages():
            timestamp = t.to_sec()
            relative_time = timestamp - experiment_start
            message_count += 1
            
            # Process SII data
            if topic == '/sii':
                sii_data.append({
                    'timestamp': timestamp,
                    'relative_time': relative_time,
                    'sii_value': msg.sii,
                    'agent_id': msg.agent_id,
                    'num_members': msg.num_members,
                    'msg_timestamp': msg.timestamp.to_sec() if msg.timestamp else timestamp
                })
                
                # Timeline data for social metrics
                if extract_timeline:
                    social_metrics_timeline.append({
                        'timestamp': timestamp,
                        'relative_time': relative_time,
                        'metric_type': 'SII',
                        'value': msg.sii,
                        'agent_id': msg.agent_id,
                        'group_id': '',  # SII doesn't have group_id
                        'num_members': msg.num_members
                    })
            
            # Process SGI data
            elif topic == '/sgi':
                sgi_data.append({
                    'timestamp': timestamp,
                    'relative_time': relative_time,
                    'sgi_value': msg.sgi,
                    'group_id': msg.group_id,
                    'num_members': msg.num_members,
                    'msg_timestamp': msg.timestamp.to_sec() if msg.timestamp else timestamp
                })
                
                # Timeline data for social metrics
                if extract_timeline:
                    social_metrics_timeline.append({
                        'timestamp': timestamp,
                        'relative_time': relative_time,
                        'metric_type': 'SGI',
                        'value': msg.sgi,
                        'agent_id': '',  # SGI doesn't have agent_id
                        'group_id': msg.group_id,
                        'num_members': msg.num_members
                    })
            
            # Process Move Base Status data
            elif topic == '/move_base/status':
                if msg.status_list:
                    # Đồng bộ với move_base_status_timeline.py: lấy phần tử đầu tiên.
                    status = msg.status_list[0]
                    move_base_status_data.append({
                        'timestamp': timestamp,
                        'relative_time': relative_time,
                        'status_code': status.status,
                        'status_name': status_names.get(status.status, "UNKNOWN"),
                        'goal_id': status.goal_id.id if status.goal_id else "",
                        'goal_stamp': status.goal_id.stamp.to_sec() if status.goal_id and status.goal_id.stamp else timestamp,
                        'status_text': status.text if status.text else ""
                    })
                    
                    # Timeline data for move base status
                    if extract_timeline:
                        move_base_timeline.append({
                            'timestamp': timestamp,
                            'relative_time': relative_time,
                            'status_code': status.status,
                            'status_name': status_names.get(status.status, "UNKNOWN"),
                            'goal_id': status.goal_id.id if status.goal_id else "",
                            'status_text': status.text if status.text else ""
                        })
            
            # Process Path Planning for timeline (always for MATLAB compatibility)
            elif topic in ('/move_base/NavfnROS/plan', '/move_base/GlobalPlanner/plan'):
                path_found = len(msg.poses) > 0
                path_planning_timeline.append({
                    'timestamp': timestamp,
                    'relative_time': relative_time,
                    'planner_topic': topic,
                    'path_found': 1 if path_found else 0,
                    'num_poses': len(msg.poses),
                    'path_length': calculate_path_length(msg.poses) if path_found else 0
                })

            # AMCL pose (navigation thật / bag không có Gazebo)
            elif topic == '/amcl_pose':
                p = msg.pose.pose
                cov = msg.pose.covariance
                data_entry = {
                    'timestamp': timestamp,
                    'robot_x': p.position.x,
                    'robot_y': p.position.y,
                    'robot_qx': p.orientation.x,
                    'robot_qy': p.orientation.y,
                    'robot_qz': p.orientation.z,
                    'robot_qw': p.orientation.w,
                }
                amcl_pose_data.append(data_entry)
                if extract_timeline:
                    robot_trajectory.append({
                        'timestamp': timestamp,
                        'relative_time': relative_time,
                        'x': p.position.x,
                        'y': p.position.y,
                        'z': p.position.z,
                        'cov_xx': cov[0] if len(cov) > 0 else '',
                        'cov_yy': cov[7] if len(cov) > 7 else '',
                    })
            
            # Process Model States data
            elif topic == '/gazebo/model_states':
                # Collect actor names
                for name in msg.name:
                    if name != 'robot' and name.startswith('actor'):
                        all_actors.add(name)
                
                # Extract robot and actors data
                robot_idx = None
                actor_indices = {}
                
                for i, name in enumerate(msg.name):
                    if name == 'robot':
                        robot_idx = i
                    elif name.startswith('actor'):
                        actor_indices[name] = i
                
                if robot_idx is not None:
                    robot_pos = msg.pose[robot_idx]
                    robot_vel = msg.twist[robot_idx]
                    data_entry = {
                        'timestamp': timestamp,
                        'robot_x': robot_pos.position.x,
                        'robot_y': robot_pos.position.y,
                        'robot_qx': robot_pos.orientation.x,
                        'robot_qy': robot_pos.orientation.y,
                        'robot_qz': robot_pos.orientation.z,
                        'robot_qw': robot_pos.orientation.w
                    }
                    
                    # Add all actors positions
                    for actor_name in sorted(all_actors):
                        if actor_name in actor_indices:
                            idx = actor_indices[actor_name]
                            pos = msg.pose[idx]
                            vel = msg.twist[idx]
                            
                            data_entry.update({
                                f'{actor_name}_x': pos.position.x,
                                f'{actor_name}_y': pos.position.y,
                                f'{actor_name}_qx': pos.orientation.x,
                                f'{actor_name}_qy': pos.orientation.y,
                                f'{actor_name}_qz': pos.orientation.z,
                                f'{actor_name}_qw': pos.orientation.w
                            })
                            
                            # Timeline data for actors trajectory
                            if extract_timeline:
                                if actor_name not in actors_trajectory:
                                    actors_trajectory[actor_name] = []
                                
                                actors_trajectory[actor_name].append({
                                    'timestamp': timestamp,
                                    'relative_time': relative_time,
                                    'x': pos.position.x,
                                    'y': pos.position.y
                                })
                        else:
                            # Actor not found, set to empty
                            data_entry.update({
                                f'{actor_name}_x': '', f'{actor_name}_y': '',
                                f'{actor_name}_qx': '', f'{actor_name}_qy': '', f'{actor_name}_qz': '', f'{actor_name}_qw': ''
                            })
                    
                    actors_data.append(data_entry)

        if not actors_data and amcl_pose_data:
            actors_data = amcl_pose_data

        # Quỹ đạo robot trong *_robot_trajectory.csv: từ AMCL ở trên, hoặc từ actors_data
        # (Gazebo hoặc fallback khi chỉ có pose đồng bộ trong CSV chính).
        if extract_timeline and not robot_trajectory and actors_data:
            for e in actors_data:
                robot_trajectory.append({
                    'timestamp': e['timestamp'],
                    'relative_time': e['timestamp'] - experiment_start,
                    'x': e['robot_x'],
                    'y': e['robot_y'],
                    'z': '',
                    'cov_xx': '',
                    'cov_yy': '',
                })
        
        bag.close()
        
        extraction_time = time.time() - start_time
        print(f"✓ Hoàn thành trích xuất trong {extraction_time:.2f}s")
        
        # Create output directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        rosbag_analysis_dir = os.path.dirname(script_dir)
        results_dir = output_dir if output_dir else os.path.join(rosbag_analysis_dir, 'results')
        os.makedirs(results_dir, exist_ok=True)
        print(f"\n=== TẠO FILE DỮ LIỆU ĐỒNG BỘ (TỐI ƯU) ===")

        # Create synchronized data with optimized algorithm
        create_synchronized_data_optimized(
            actors_data,
            sii_data,
            sgi_data,
            move_base_status_data,
            path_planning_timeline,
            results_dir,
            output_csv_name,
            bag_basename,
            experiment_start,
        )
        create_matlab_data_config(results_dir, output_csv_name)
        
        # Save timeline data if enabled
        if extract_timeline:
            print(f"\n=== LƯU DỮ LIỆU TIMELINE ===")
            
            # 1. Move Base Status Timeline
            if move_base_timeline:
                save_timeline_csv(move_base_timeline, results_dir, f"{bag_basename}_move_base_timeline.csv", 
                                ['timestamp', 'relative_time', 'status_code', 'status_name', 'goal_id', 'status_text'])
                print(f"✅ Move Base Timeline: {len(move_base_timeline)} entries")
            
            # 2. Path Planning Timeline
            if path_planning_timeline:
                save_timeline_csv(path_planning_timeline, results_dir, f"{bag_basename}_path_planning_timeline.csv",
                                ['timestamp', 'relative_time', 'planner_topic', 'path_found', 'num_poses', 'path_length'])
                print(f"✅ Path Planning Timeline: {len(path_planning_timeline)} entries")
            
            # 3. Robot Trajectory
            if robot_trajectory:
                save_timeline_csv(robot_trajectory, results_dir, f"{bag_basename}_robot_trajectory.csv",
                                ['timestamp', 'relative_time', 'x', 'y', 'z', 'cov_xx', 'cov_yy'])
                print(f"✅ Robot Trajectory: {len(robot_trajectory)} entries")
            
            # 4. Social Metrics Timeline
            if social_metrics_timeline:
                save_timeline_csv(social_metrics_timeline, results_dir, f"{bag_basename}_social_metrics_timeline.csv",
                                ['timestamp', 'relative_time', 'metric_type', 'value', 'agent_id', 'group_id', 'num_members'])
                print(f"✅ Social Metrics Timeline: {len(social_metrics_timeline)} entries")
            
            # 5. Actors Trajectories
            for actor_name, trajectory in actors_trajectory.items():
                if trajectory:
                    filename = f"{bag_basename}_{actor_name}_trajectory.csv"
                    save_timeline_csv(trajectory, results_dir, filename,
                                    ['timestamp', 'relative_time', 'x', 'y'])
                    print(f"✅ {actor_name} Trajectory: {len(trajectory)} entries")
            
            # Create visualization script
            create_visualization_script(results_dir, bag_basename)
        
        total_time = time.time() - start_time
        print(f"\n🎉 Hoàn thành trong {total_time:.2f}s (tối ưu)")
        
        return True
        
    except Exception as e:
        print(f"❌ Lỗi trong quá trình trích xuất: {e}")
        return False

def calculate_path_length(poses):
    """Tính độ dài của path"""
    if len(poses) < 2:
        return 0
    
    total_length = 0
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i-1].pose.position.x
        dy = poses[i].pose.position.y - poses[i-1].pose.position.y
        total_length += (dx**2 + dy**2)**0.5
    
    return total_length

def save_timeline_csv(data, results_dir, filename, fieldnames):
    """Lưu dữ liệu timeline vào CSV"""
    output_file = os.path.join(results_dir, filename)
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)

def create_visualization_script(results_dir, bag_basename):
    """Tạo script visualization cho timeline data"""
    
    viz_script = f"""#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Load timeline data
results_dir = '{results_dir}'
bag_basename = '{bag_basename}'

# Load data
try:
    move_base_data = pd.read_csv(f'{{results_dir}}/{{bag_basename}}_move_base_timeline.csv')
    print("✅ Loaded Move Base Timeline data")
except:
    move_base_data = None
    print("❌ Move Base Timeline data not found")

try:
    path_data = pd.read_csv(f'{{results_dir}}/{{bag_basename}}_path_planning_timeline.csv')
    print("✅ Loaded Path Planning Timeline data")
except:
    path_data = None
    print("❌ Path Planning Timeline data not found")

try:
    robot_data = pd.read_csv(f'{{results_dir}}/{{bag_basename}}_robot_trajectory.csv')
    print("✅ Loaded Robot Trajectory data")
except:
    robot_data = None
    print("❌ Robot Trajectory data not found")

try:
    social_data = pd.read_csv(f'{{results_dir}}/{{bag_basename}}_social_metrics_timeline.csv')
    print("✅ Loaded Social Metrics Timeline data")
except:
    social_data = None
    print("❌ Social Metrics Timeline data not found")

# Create visualization
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.suptitle(f'Timeline Analysis - {{bag_basename}}', fontsize=16, fontweight='bold')

# 1. Move Base Status Timeline
if move_base_data is not None:
    ax1 = axes[0, 0]
    ax1.plot(move_base_data['relative_time'], move_base_data['status_code'], 'b-', linewidth=2, marker='o', markersize=3)
    ax1.set_title('Move Base Status Timeline')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Status Code')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-1, 10)
    ax1.set_yticks(range(10))
    ax1.set_yticklabels(['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 
                        'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'])

# 2. Path Planning Timeline
if path_data is not None:
    ax2 = axes[0, 1]
    ax2.plot(path_data['relative_time'], path_data['path_found'], 'g-', linewidth=2, marker='o', markersize=3)
    ax2.set_title('Path Planning Status')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Path Found (0/1)')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-0.1, 1.1)

# 3. Robot Trajectory
if robot_data is not None:
    ax3 = axes[1, 0]
    ax3.plot(robot_data['x'], robot_data['y'], 'r-', linewidth=2, alpha=0.7)
    ax3.set_title('Robot Trajectory')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')

# 4. Social Metrics Timeline
if social_data is not None:
    ax4 = axes[1, 1]
    sii_data = social_data[social_data['metric_type'] == 'SII']
    sgi_data = social_data[social_data['metric_type'] == 'SGI']
    
    if not sii_data.empty:
        ax4.plot(sii_data['relative_time'], sii_data['value'], 'b-', linewidth=2, label='SII', marker='o', markersize=3)
    if not sgi_data.empty:
        ax4.plot(sgi_data['relative_time'], sgi_data['value'], 'g-', linewidth=2, label='SGI', marker='s', markersize=3)
    
    ax4.set_title('Social Metrics Timeline')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Metric Value')
    ax4.grid(True, alpha=0.3)
    ax4.legend()

plt.tight_layout()
plt.show()

print("\\n📊 Timeline visualization completed!")
"""
    
    viz_file = os.path.join(results_dir, f"{bag_basename}_timeline_visualization.py")
    with open(viz_file, 'w') as f:
        f.write(viz_script)
    
    # Make it executable
    os.chmod(viz_file, 0o755)
    print(f"✅ Tạo script visualization: {viz_file}")

def create_synchronized_data_optimized(
    actors_data,
    sii_data,
    sgi_data,
    move_base_status_data,
    path_planning_data,
    results_dir,
    output_csv_name,
    experiment_name,
    experiment_start,
):
    """Tạo file dữ liệu đồng bộ với thuật toán tối ưu."""
    
    print(f"Tạo file dữ liệu đồng bộ với thuật toán tối ưu...")
    
    # Create sorted lists for time synchronization
    sii_timestamps = sorted([d['timestamp'] for d in sii_data])
    sgi_timestamps = sorted([d['timestamp'] for d in sgi_data])
    move_base_timestamps = sorted([d['timestamp'] for d in move_base_status_data])
    path_planning_timestamps = sorted([d['timestamp'] for d in path_planning_data]) if path_planning_data else []
    
    # Create dictionaries for quick lookup
    sii_dict = {d['timestamp']: d for d in sii_data}
    sgi_dict = {d['timestamp']: d for d in sgi_data}
    move_base_dict = {d['timestamp']: d for d in move_base_status_data}
    path_planning_dict = {d['timestamp']: d for d in path_planning_data} if path_planning_data else {}
    
    # Time-synchronized sampling policy:
    # - Forward-fill: giữ giá trị mới nhất có timestamp <= thời điểm hiện tại
    # - Backfill ở đầu chuỗi: nếu chưa có giá trị quá khứ thì dùng giá trị đầu tiên
    # Mục tiêu: luôn có dữ liệu liên tục theo trục thời gian để vẽ đồ thị.
    sii_idx = 0
    sgi_idx = 0
    move_base_idx = 0
    path_planning_idx = 0
    last_sii_ts = sii_timestamps[0] if sii_timestamps else None
    last_sgi_ts = sgi_timestamps[0] if sgi_timestamps else None
    last_move_base_ts = move_base_timestamps[0] if move_base_timestamps else None
    last_path_planning_ts = path_planning_timestamps[0] if path_planning_timestamps else None
    
    # Use actors_data as base since it has the most frequent updates
    synchronized_data = []
    
    print(f"  Đang đồng bộ {len(actors_data)} data entries với binary search...")
    
    for i, actor_entry in enumerate(actors_data):
        timestamp = actor_entry['timestamp']
        relative_time = timestamp - experiment_start
        
        # Advance pointers and use latest known sample (forward-fill on unified timestamp axis)
        while sii_idx < len(sii_timestamps) and sii_timestamps[sii_idx] <= timestamp:
            last_sii_ts = sii_timestamps[sii_idx]
            sii_idx += 1
        while sgi_idx < len(sgi_timestamps) and sgi_timestamps[sgi_idx] <= timestamp:
            last_sgi_ts = sgi_timestamps[sgi_idx]
            sgi_idx += 1
        while move_base_idx < len(move_base_timestamps) and move_base_timestamps[move_base_idx] <= timestamp:
            last_move_base_ts = move_base_timestamps[move_base_idx]
            move_base_idx += 1
        while path_planning_idx < len(path_planning_timestamps) and path_planning_timestamps[path_planning_idx] <= timestamp:
            last_path_planning_ts = path_planning_timestamps[path_planning_idx]
            path_planning_idx += 1
        
        # Create synchronized entry
        sync_entry = actor_entry.copy()
        
        # Add relative_time for MATLAB compatibility
        sync_entry['relative_time'] = relative_time
        
        # Add SII data
        if last_sii_ts is not None:
            sii_entry = sii_dict[last_sii_ts]
            sync_entry.update({
                'sii_value': sii_entry['sii_value'],
                'sii_agent_id': sii_entry['agent_id'],
                'sii_num_members': sii_entry['num_members'],
                'sii_relative_time': sii_entry.get('relative_time', relative_time)
            })
        else:
            sync_entry.update({
                'sii_value': '', 'sii_agent_id': '', 'sii_num_members': '', 'sii_relative_time': ''
            })
        
        # Add SGI data
        if last_sgi_ts is not None:
            sgi_entry = sgi_dict[last_sgi_ts]
            sync_entry.update({
                'sgi_value': sgi_entry['sgi_value'],
                'sgi_group_id': sgi_entry['group_id'],
                'sgi_num_members': sgi_entry['num_members'],
                'sgi_relative_time': sgi_entry.get('relative_time', relative_time)
            })
        else:
            sync_entry.update({
                'sgi_value': '', 'sgi_group_id': '', 'sgi_num_members': '', 'sgi_relative_time': ''
            })
        
        # Add Move Base Status data (with MATLAB compatible headers)
        if last_move_base_ts is not None:
            move_base_entry = move_base_dict[last_move_base_ts]
            sync_entry.update({
                'move_base_status_code': move_base_entry['status_code'],
                'move_base_status_name': move_base_entry['status_name'],
                'move_base_goal_id': move_base_entry['goal_id'],
                'move_base_goal_stamp': move_base_entry['goal_stamp'],
                'move_base_status_text': move_base_entry['status_text'],
                # MATLAB compatible headers
                'status_code': move_base_entry['status_code'],  # For plot_move_base_from_csv.m
                'status_name': move_base_entry['status_name']   # For plot_move_base_from_csv.m
            })
        else:
            sync_entry.update({
                'move_base_status_code': '', 'move_base_status_name': '', 'move_base_goal_id': '',
                'move_base_goal_stamp': '', 'move_base_status_text': '',
                'status_code': '', 'status_name': ''  # MATLAB compatible
            })
        
        # Add Path Planning data (with MATLAB compatible headers)
        if last_path_planning_ts is not None:
            path_planning_entry = path_planning_dict[last_path_planning_ts]
            sync_entry.update({
                'path_found': path_planning_entry['path_found'],
                'num_poses': path_planning_entry['num_poses'],
                'path_length': path_planning_entry['path_length'],
                'planner_topic': path_planning_entry.get('planner_topic', '')
            })
        else:
            sync_entry.update({
                'path_found': '', 'num_poses': '', 'path_length': '', 'planner_topic': ''
            })
        
        synchronized_data.append(sync_entry)
        
    # Save synchronized data with optimized CSV writing
    # CSV chính chỉ giữ các cột tối giản theo yêu cầu và dùng chung 1 trục thời gian `timestamp`.
    if synchronized_data:
        header = ['timestamp']
        
        # Add robot model states headers
        header.extend(['robot_x', 'robot_y', 'robot_qx', 'robot_qy', 'robot_qz', 'robot_qw'])
        
        # Add actor headers
        all_actors = set()
        for entry in synchronized_data:
            for key in entry.keys():
                if key.startswith('actor') and key.endswith('_x'):
                    all_actors.add(key.replace('_x', ''))
        
        for actor in sorted(all_actors):
            header.extend([f'{actor}_x', f'{actor}_y', 
                          f'{actor}_qx', f'{actor}_qy', f'{actor}_qz', f'{actor}_qw'])
        
        # Add minimal requested metrics headers
        header.extend(['sii_value', 'sgi_value', 'move_base_status_code', 'path_found'])
        
        print(f"  Đang xử lý {len(synchronized_data)} dòng đồng bộ...")
        print(f"  Headers: {len(header)} columns")
        
        # Prepare all rows in memory first
        rows = []
        for i, data in enumerate(synchronized_data):
            row = [data['timestamp']]

            # Add robot model states
            row.extend([data['robot_x'], data['robot_y'], data['robot_qx'], data['robot_qy'], data['robot_qz'], data['robot_qw']])

            # Add actors data
            for actor in sorted(all_actors):
                row.extend([data.get(f'{actor}_x', ''), data.get(f'{actor}_y', ''),
                           data.get(f'{actor}_qx', ''), data.get(f'{actor}_qy', ''), data.get(f'{actor}_qz', ''), data.get(f'{actor}_qw', '')])

            # Add minimal requested metrics
            row.extend([data['sii_value'], data['sgi_value'], data['move_base_status_code'], data['path_found']])

            rows.append(row)

        output_file = os.path.join(results_dir, output_csv_name)
        with open(output_file, 'w', newline='', buffering=8192) as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        print(f"✓ Đã lưu {len(synchronized_data)} synchronized data entries vào {output_file}")

def create_matlab_data_config(results_dir, output_csv_name):
    """Tạo data_config.m trong results để quick_start nhận đúng file CSV."""
    output_csv_path = os.path.join(results_dir, output_csv_name)
    config_file = os.path.join(results_dir, 'data_config.m')

    config_content = f"""% Auto-generated by extract_experiment_optimized.py
% MATLAB sẽ dùng trực tiếp CSV đồng bộ trong rosbag_analysis/results.

data_path = '{results_dir}';
robot_file = fullfile(data_path, '{output_csv_name}');
human_file = fullfile(data_path, '{output_csv_name}');
social_file = fullfile(data_path, '{output_csv_name}');
system_file = fullfile(data_path, '{output_csv_name}');
robot_actors_file = fullfile(data_path, '{output_csv_name}');

figure1_file = robot_actors_file;  % Môi trường + quỹ đạo robot/actors
figure2_file = social_file;        % SII timeline
figure3_file = social_file;        % SGI timeline
figure4_file = system_file;        % Move base status timeline
figure5_file = system_file;        % Path planning timeline
figure6_file = robot_actors_file;  % Robot and actors trajectory
"""

    with open(config_file, 'w', encoding='utf-8') as f:
        f.write(config_content)

    if os.path.isfile(output_csv_path):
        print(f"✓ Đã tạo MATLAB config: {config_file}")
    else:
        print(f"⚠️ Đã tạo config nhưng chưa thấy CSV mục tiêu: {output_csv_path}")

if __name__ == "__main__":
    print("=== CHƯƠNG TRÌNH TRÍCH XUẤT THÍ NGHIỆM (TỐI ƯU) ===")
    print("Trích xuất dữ liệu từ ROS bag file với tối ưu hóa hiệu suất:")
    print("• Đọc bag file 1 lần duy nhất")
    print("• Sử dụng binary search cho tìm kiếm timestamp")
    print("• Tối ưu I/O với buffering")
    print("• Actors positions, Social metrics (SII, SGI)")
    print("• Ghi file CSV tổng hợp trong results/")
    print()
    
    # Parse command line arguments
    bag_file = None
    extract_timeline = False
    output_dir = None
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "--timeline" or arg == "-t":
            extract_timeline = True
            print("➕ Timeline extraction enabled")
            i += 1
            continue
        elif arg == "--output-dir" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]
            print(f"📁 Output dir: {output_dir}")
            i += 2
            continue
        elif arg.startswith("-"):
            print(f"❌ Unknown option: {arg}")
            print("Usage: python3 extract_experiment_optimized.py [bag_file] [--timeline] [--output-dir DIR]")
            sys.exit(1)
        else:
            if bag_file is None:
                bag_file = arg
            else:
                print(f"❌ Multiple bag files specified: {bag_file} and {arg}")
                sys.exit(1)
        i += 1
    
    # Use default bag file if none specified
    if bag_file is None:
        # Default to the most recent bag file
        bag_file = "data/system_data_20250720_111518.bag"
        print(f"Sử dụng bag file mặc định: {bag_file}")
    else:
        print(f"Sử dụng bag file: {bag_file}")
    
    if extract_timeline:
        print("📈 Timeline extraction mode: Will generate timeline CSV files and visualization script")
    
    print()
    success = extract_experiment_data_optimized(bag_file, extract_timeline, output_dir=output_dir)
    if success:
        print(f"\n🎉 Trích xuất thí nghiệm tối ưu hoàn tất thành công!")
        if output_dir:
            print(f"📊 CSV tổng hợp: {output_dir}/")
        else:
            print("📊 CSV tổng hợp: results/ (mặc định)")
        if extract_timeline:
            print(f"📈 Timeline data và visualization script đã được tạo")
    else:
        print(f"\n❌ Trích xuất thí nghiệm tối ưu thất bại!")
        print(f"🔍 Vui lòng kiểm tra lại bag file và thử lại.") 