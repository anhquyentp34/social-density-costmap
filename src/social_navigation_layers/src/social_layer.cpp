#include <social_navigation_layers/social_layer.h> // Thư vien social_layer
#include <math.h> // Thư vien toan hoc
#include <angles/angles.h> // Thư vien goc
#include <pluginlib/class_list_macros.h> // Thư vien pluginlib
#include <geometry_msgs/PointStamped.h> // Thư vien geometry_msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Thư vien tf2
#include <algorithm> // Thư vien thuat toan
#include <string> // Thư vien chuoi

using costmap_2d::NO_INFORMATION; // Su dung bien NO_INFORMATION tu thu vien costmap_2d
using costmap_2d::LETHAL_OBSTACLE; // Su dung bien LETHAL_OBSTACLE tu thu vien costmap_2d
using costmap_2d::FREE_SPACE; // Su dung bien FREE_SPACE tu thu vien costmap_2d

namespace social_navigation_layers // Khong gian ten social_navigation_layers
{
    void SocialLayer::onInitialize() // Ham khoi tao
    {
        ros::NodeHandle nh("~/" + name_); // Tao node handle
        current_ = true; // Dat gia tri current_ la true
        first_time_ = true; // Dat gia tri first_time_ la true
        social_sub_ = nh.subscribe("/social_state", 1, &SocialLayer::socialCallback, this); // Dang ky subscriber cho topic /human_information
    }
    
    void SocialLayer::socialCallback(const social_msgs::SocialState& socialstate_msgs) { // Ham callback khi nhan du lieu tu topic
        boost::recursive_mutex::scoped_lock lock(lock_); // Khoa de dam bao an toan luong
        const ros::Time now = ros::Time::now();
        const double keep_sec = people_keep_time_.toSec();

        // Tạo danh sách cục bộ từ message mới
        social_msgs::SocialPeople social_people_list;
        social_msgs::SocialGroups social_groups_list;
        social_msgs::SocialInteractions social_interactions_list;
        
        social_people_list.header = socialstate_msgs.header;
        social_groups_list.header = socialstate_msgs.header;
        social_interactions_list.header = socialstate_msgs.header;

        for(unsigned int i=0; i<socialstate_msgs.people.people.size(); i++)
        {
          social_people_list.people.push_back(socialstate_msgs.people.people[i]);
        }

        for(unsigned int i=0; i<socialstate_msgs.groups.groups.size(); i++)
        {
          social_groups_list.groups.push_back(socialstate_msgs.groups.groups[i]);
        }

        for(unsigned int i=0; i<socialstate_msgs.interactions.interactions.size(); i++)
        {
          social_interactions_list.interactions.push_back(socialstate_msgs.interactions.interactions[i]);
        }

        // Hold-last: mất track ZED vẫn publish /social_state rỗng mỗi chu kỳ.
        // Nếu list mới rỗng nhưng còn trong keep_time kể từ lần non-empty cuối → giữ bản cũ.
        auto still_holding = [now, keep_sec](const ros::Time& last_nonempty) -> bool {
            if (keep_sec < 0.0 || last_nonempty.isZero())
                return false;
            return (now - last_nonempty).toSec() <= keep_sec;
        };

        const bool fresh_data = !social_people_list.people.empty()
            || !social_groups_list.groups.empty()
            || !social_interactions_list.interactions.empty();

        if (!social_people_list.people.empty()) {
            social_people_list_ = social_people_list;
            last_people_nonempty_ = now;
        } else if (!still_holding(last_people_nonempty_)) {
            social_people_list_ = social_people_list;  // thật sự xóa
        }
        // else: giữ social_people_list_ cũ

        if (!social_groups_list.groups.empty()) {
            social_groups_list_ = social_groups_list;
            last_groups_nonempty_ = now;
        } else if (!still_holding(last_groups_nonempty_)) {
            social_groups_list_ = social_groups_list;
        }

        if (!social_interactions_list.interactions.empty()) {
            social_interactions_list_ = social_interactions_list;
            last_interactions_nonempty_ = now;
        } else if (!still_holding(last_interactions_nonempty_)) {
            social_interactions_list_ = social_interactions_list;
        }

        // Density: cập nhật khi có data mới; giữ cũ khi đang hold; hết hold → 0
        if (fresh_data) {
            density_ = std::max(0.0, socialstate_msgs.density);
        } else if (!still_holding(last_people_nonempty_)
                   && !still_holding(last_groups_nonempty_)
                   && !still_holding(last_interactions_nonempty_)) {
            density_ = 0.0;
        }

        last_social_update_ = now;
    }

    void SocialLayer::expireStaleSocialCaches()
    {
        const ros::Time now = ros::Time::now();
        const double keep_sec = people_keep_time_.toSec();
        if (keep_sec < 0.0)
            return;

        auto expired = [now, keep_sec](const ros::Time& last_nonempty) -> bool {
            if (last_nonempty.isZero())
                return true;
            return (now - last_nonempty).toSec() > keep_sec;
        };

        if (expired(last_people_nonempty_)) {
            social_people_list_.people.clear();
        }
        if (expired(last_groups_nonempty_)) {
            social_groups_list_.groups.clear();
        }
        if (expired(last_interactions_nonempty_)) {
            social_interactions_list_.interactions.clear();
        }

        // Không còn social message nào trong keep_time → xóa hết + density
        if (!last_social_update_.isZero() &&
            (now - last_social_update_).toSec() > keep_sec) {
            social_people_list_.people.clear();
            social_groups_list_.groups.clear();
            social_interactions_list_.interactions.clear();
            density_ = 0.0;
        } else if (social_people_list_.people.empty()
                   && social_groups_list_.groups.empty()
                   && social_interactions_list_.interactions.empty()) {
            density_ = 0.0;
        }
    }

    void SocialLayer::updateBounds(double origin_x, double origin_y, double origin_z, 
                                    double* min_x, double* min_y, double* max_x, double* max_y){
        boost::recursive_mutex::scoped_lock lock(lock_); // Khoa de dam bao an toan luong

        expireStaleSocialCaches();
        
        std::string global_frame = layered_costmap_->getGlobalFrameID(); // Lay ID khung toa do toan cau
        //std::cout << "GlobalFrameID="<<global_frame<<std::endl;
        transformed_people_.clear(); // Xoa danh sach thong tin nguoi da bien doi
        
        for(unsigned int i=0; i<social_people_list_.people.size(); i++)
        {
            social_msgs::SocialPerson& person = social_people_list_.people[i]; // Lay thong tin cua tung nguoi
           
            social_msgs::SocialPerson tpt; // Thong tin nguoi da bien doi
            
            tpt.name = person.name;  // Đảm bảo sao chép tên
            geometry_msgs::PointStamped pt, opt; // Cac diem toa do
            
            try{
              pt.point.x = person.position.position.x; // Gan toa do x
              pt.point.y = person.position.position.y; // Gan toa do y
              pt.point.z = person.position.position.z; // Gan toa do z
              pt.header.frame_id = social_people_list_.header.frame_id; // Gan ID khung toa do
              
              tf_->transform(pt, opt, global_frame); // Bien doi toa do
              tpt.position.position.x = opt.point.x; // Gan toa do x da bien doi
              tpt.position.position.y = opt.point.y; // Gan toa do y da bien doi
              tpt.position.position.z = opt.point.z; // Gan toa do z da bien doi


              pt.point.x += person.velocity.linear.x; // Gan van toc x
              pt.point.y += person.velocity.linear.y; // Gan van toc y
              pt.point.z += person.velocity.linear.z; // Gan van toc z
              
              tf_->transform(pt, opt, global_frame); // Bien doi toa do voi van toc
              
              tpt.velocity.linear.x = opt.point.x - tpt.position.position.x;
              tpt.velocity.linear.y = opt.point.y - tpt.position.position.y;
              tpt.velocity.linear.z = opt.point.z - tpt.position.position.z;

            
              tpt.reliability = person.reliability; // Gan do tin cay
              tpt.emotion = person.emotion; // Sao chép trường emotion

              transformed_people_.push_back(tpt); // Them vao danh sach thong tin nguoi da bien doi
              
            }
            catch(tf2::LookupException& ex) {
              ROS_ERROR("No Transform available Error: %s\n", ex.what()); // Xu ly loi khong tim thay bien doi
              continue;
            }
            catch(tf2::ConnectivityException& ex) {
              ROS_ERROR("Connectivity Error: %s\n", ex.what()); // Xu ly loi ket noi
              continue;
            }
            catch(tf2::ExtrapolationException& ex) {
              ROS_ERROR("Extrapolation Error: %s\n", ex.what()); // Xu ly loi ngoai suy
              continue;
            }
        }

      

        updateBoundsFromPeople(min_x, min_y, max_x, max_y); // Cap nhat gioi han tu thong tin nguoi
        if(first_time_){
            last_min_x_ = *min_x; // Gan gia tri gioi han x cuoi cung
            last_min_y_ = *min_y;    
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;    
            first_time_ = false; // Dat lai bien first_time_
        }else{
            double a = *min_x, b = *min_y, c = *max_x, d = *max_y; // Luu gia tri gioi han hien tai
            *min_x = std::min(last_min_x_, *min_x); // Cap nhat gia tri min_x
            *min_y = std::min(last_min_y_, *min_y);
            *max_x = std::max(last_max_x_, *max_x);
            *max_y = std::max(last_max_y_, *max_y);
            last_min_x_ = a; // Gan lai gia tri gioi han cuoi cung
            last_min_y_ = b;
            last_max_x_ = c;
            last_max_y_ = d;
        
        }
        
    }

};
