# SocialInteraction.msg
string object_name
geometry_msgs/Pose object_position
geometry_msgs/Twist object_velocity
float64 reliability # độ chính xác
social_msgs/SocialPerson[] participants # danh sách người tham gia tương tác
string interaction_type # loại tương tác (ví dụ: nhìn, chạm, nói chuyện)
time timestamp # thời gian của tương tác
# thêm các thuộc tính khác nếu cần!
