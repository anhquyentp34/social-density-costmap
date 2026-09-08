#include <social_navigation_layers/human_group_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <cmath>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;

namespace social_navigation_layers
{
    namespace
    {
        double computeDensityGeometryScale(double density, double density_weight)
        {
            const double safe_density = std::max(0.0, density);
            return std::exp(-density_weight * safe_density);
        }
    }  // namespace

    // Structure to hold circle parameters
    struct Circle {
        double a;  // X-coordinate of center
        double b;  // Y-coordinate of center
        double r;  // Radius
        double s;  // Root mean square error
        int j;     // Number of iterations
    };

    // Structure to hold data points
    struct Data {
        std::vector<double> X;
        std::vector<double> Y;
        double meanX;
        double meanY;
        int n;

        void means() {
            meanX = 0.0;
            meanY = 0.0;
            for(int i = 0; i < n; i++) {
                meanX += X[i];
                meanY += Y[i];
            }
            meanX /= n;
            meanY /= n;
        }
    };

    // Constants for circle fitting
    const double Four = 4.0;
    const double Three = 3.0;
    const double Two = 2.0;

    // Circle fitting function using Taubin's method
    Circle CircleFitByTaubin(Data& data) {
        // Special handling for 2 people
        if (data.n == 2) {
            Circle circle;
            // Calculate midpoint (center)
            circle.a = (data.X[0] + data.X[1]) / 2.0;
            circle.b = (data.Y[0] + data.Y[1]) / 2.0;
            // Calculate radius as half the distance between points
            double dx = data.X[1] - data.X[0];
            double dy = data.Y[1] - data.Y[0];
            circle.r = std::sqrt(dx*dx + dy*dy) / 2.0;
            circle.j = 0;  // No iterations needed for 2 points
            return circle;
        }

        // Original Taubin method for 3 or more points
        int i, iter, IterMAX = 99;
        
        double Xi, Yi, Zi;
        double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
        double A0, A1, A2, A22, A3, A33;
        double Dy, xnew, x, ynew, y;
        double DET, Xcenter, Ycenter;
        
        Circle circle;
        
        data.means();   // Compute x- and y- sample means

        // Computing moments
        Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.0;
        
        for (i = 0; i < data.n; i++) {
            Xi = data.X[i] - data.meanX;   // centered x-coordinates
            Yi = data.Y[i] - data.meanY;   // centered y-coordinates
            Zi = Xi*Xi + Yi*Yi;
            
            Mxy += Xi*Yi;
            Mxx += Xi*Xi;
            Myy += Yi*Yi;
            Mxz += Xi*Zi;
            Myz += Yi*Zi;
            Mzz += Zi*Zi;
        }
        Mxx /= data.n;
        Myy /= data.n;
        Mxy /= data.n;
        Mxz /= data.n;
        Myz /= data.n;
        Mzz /= data.n;
        
        // Computing coefficients of the characteristic polynomial
        Mz = Mxx + Myy;
        Cov_xy = Mxx*Myy - Mxy*Mxy;
        Var_z = Mzz - Mz*Mz;
        A3 = Four*Mz;
        A2 = -Three*Mz*Mz - Mzz;
        A1 = Var_z*Mz + Four*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
        A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
        A22 = A2 + A2;
        A33 = A3 + A3 + A3;

        // Finding the root of the characteristic polynomial using Newton's method
        for (x = 0.0, y = A0, iter = 0; iter < IterMAX; iter++) {
            Dy = A1 + x*(A22 + A33*x);
            xnew = x - y/Dy;
            if ((xnew == x) || (!std::isfinite(xnew))) break;
            ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
            if (std::abs(ynew) >= std::abs(y)) break;
            x = xnew;  y = ynew;
        }
        
        // Computing parameters of the fitting circle
        DET = x*x - x*Mz + Cov_xy;
        Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/Two;
        Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/Two;

        // Assembling the output
        circle.a = Xcenter + data.meanX;
        circle.b = Ycenter + data.meanY;
        circle.r = std::sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);
        circle.j = iter;
        
        return circle;
    }

    void HumanGroupLayer::onInitialize() {
        SocialLayer::onInitialize();
        current_ = true;
        enabled_ = true;
        
        ros::NodeHandle nh("~/" + name_);
        
        // Initialize Gaussian distribution parameters
        cutoff_ = 0.8;      // Base threshold for cost calculation
        amplitude_ = 254.0;  // Maximum cost at group center
        covar_ = 4.0;       // Base covariance for Gaussian
        factor_ = 1.0;      // Factor to adjust group radius
        
        // Setup dynamic reconfigure
        dsrv_ = boost::make_shared<dynamic_reconfigure::Server<HumanGroupLayerConfig> >(nh);
        dynamic_reconfigure::Server<HumanGroupLayerConfig>::CallbackType cb = boost::bind(&HumanGroupLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        ROS_INFO("HumanGroupLayer initialized with Gaussian parameters - cutoff=%.2f, amplitude=%.2f, covar=%.2f, factor=%.2f", 
                 cutoff_, amplitude_, covar_, factor_);
    }

    void HumanGroupLayer::reconfigureCB(HumanGroupLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;  // Giữ nguyên giá trị amplitude từ cấu hình
        covar_ = config.covar;
        factor_ = config.factor;
        density_weight_ = config.human_group_density_weight;
        people_keep_time_ = ros::Duration(config.keep_time);
    }

    // Helper function to calculate Gaussian distribution
    double HumanGroupLayer::gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew) {
        double dx = x - x0, dy = y - y0;
        double h = sqrt(dx*dx + dy*dy);
        double angle = atan2(dy, dx);
        double mx = cos(angle - skew) * h;
        double my = sin(angle - skew) * h;
        double f1 = pow(cos(skew)/varx, 2) + pow(sin(skew)/vary, 2);
        double f2 = 2*sin(2*skew)*(-1/varx + 1/vary)/4;
        double f3 = pow(sin(skew)/varx, 2) + pow(cos(skew)/vary, 2);
        return A * exp(-(mx*mx*f1 + 2*mx*my*f2 + my*my*f3));
    }

    // Helper function to calculate radius based on Gaussian parameters
    double HumanGroupLayer::get_radius(double cutoff, double amplitude, double covar) {
        return sqrt(-2 * covar * log(cutoff/amplitude));
    }

    social_msgs::SocialGroup HumanGroupLayer::analyzeSocialGroup(const social_msgs::SocialGroup& group) {
        social_msgs::SocialGroup result_group = group;
        result_group.members.clear();

        if (group.members.size() < 2) {
            ROS_DEBUG("HumanGroupLayer: Group has less than 2 members, skipping");
            result_group.radius = -1;
            return result_group;
        }

        // Prepare data for circle fitting
        Data data;
        data.n = group.members.size();
        data.X.resize(data.n);
        data.Y.resize(data.n);

        // Extract positions of group members
        for (size_t i = 0; i < group.members.size(); i++) {
            const auto& pos = group.members[i].position.position;
            data.X[i] = pos.x;
            data.Y[i] = pos.y;
            result_group.members.push_back(group.members[i]);
        }

        // Fit circle to the group members
        Circle circle = CircleFitByTaubin(data);

        // Update group parameters
        result_group.position.position.x = circle.a;
        result_group.position.position.y = circle.b;
        result_group.radius = circle.r;

        ROS_DEBUG("HumanGroupLayer: Group analyzed - Center: (%.2f, %.2f), Radius: %.2f", 
                 circle.a, circle.b, circle.r);

        return result_group;
    }

    void HumanGroupLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        expireStaleSocialCaches();
        if (!enabled_ || social_groups_list_.groups.empty()) {
            return;
        }

        std::string global_frame = layered_costmap_->getGlobalFrameID();

        for (const auto& group : social_groups_list_.groups) {
            social_msgs::SocialGroup result_group = analyzeSocialGroup(group);
            if (result_group.radius <= 0) continue;
            const double density_scale = computeDensityGeometryScale(density_, density_weight_);
            const double effective_radius = result_group.radius * density_scale;
            if (effective_radius <= 0.0) continue;

            geometry_msgs::PointStamped in_pt, out_pt;
            try {
                in_pt.header.frame_id = social_groups_list_.header.frame_id;
                in_pt.point = result_group.position.position;
                tf_->transform(in_pt, out_pt, global_frame);

                // Calculate influence radius based on group radius and Gaussian parameters
                double gaussian_radius = get_radius(cutoff_, amplitude_, covar_);
                double bound_radius = std::max(effective_radius, gaussian_radius * density_scale);
                
                // // Add safety margin to ensure complete coverage
                // bound_radius *= 1.5;

                *min_x = std::min(*min_x, out_pt.point.x - bound_radius);
                *min_y = std::min(*min_y, out_pt.point.y - bound_radius);
                *max_x = std::max(*max_x, out_pt.point.x + bound_radius);
                *max_y = std::max(*max_y, out_pt.point.y + bound_radius);
            }
            catch(tf2::LookupException& ex) {
                ROS_ERROR("HumanGroupLayer: No Transform available Error: %s", ex.what());
                continue;
            }
            catch(tf2::ConnectivityException& ex) {
                ROS_ERROR("HumanGroupLayer: Connectivity Error: %s", ex.what());
                continue;
            }
            catch(tf2::ExtrapolationException& ex) {
                ROS_ERROR("HumanGroupLayer: Extrapolation Error: %s", ex.what());
                continue;
            }
        }
    }

    void HumanGroupLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        expireStaleSocialCaches();
        if (!enabled_ || social_groups_list_.groups.empty()) {
            return;
        }

        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();

        for (const auto& group : social_groups_list_.groups) {
            social_msgs::SocialGroup result_group = analyzeSocialGroup(group);
            if (result_group.radius <= 0) continue;
            const double density_scale = computeDensityGeometryScale(density_, density_weight_);
            const double effective_radius = result_group.radius * density_scale;
            if (effective_radius <= 0.0) continue;

            geometry_msgs::PointStamped in_pt, out_pt;
            try {
                in_pt.header.frame_id = social_groups_list_.header.frame_id;
                in_pt.point = result_group.position.position;
                tf_->transform(in_pt, out_pt, layered_costmap_->getGlobalFrameID());

                unsigned int mx, my;
                if(!costmap->worldToMap(out_pt.point.x, out_pt.point.y, mx, my)) {
                    continue;
                }

                // Calculate base radius from group radius with factor
                double base = effective_radius * factor_;
                double point = base;
                
                // Calculate covar_ dynamically to ensure cost at group boundary equals cutoff_
                // Using the formula: covar = (r^2) / (-2 * ln(cutoff/amplitude))
                // where r is the group radius
                double tmp_covar = (effective_radius * effective_radius) / (-2 * log(cutoff_/amplitude_));

                unsigned int width = std::max(1, int((base + point) / res));
                unsigned int height = std::max(1, int((base + point) / res));

                double cx = out_pt.point.x, cy = out_pt.point.y;
                double angle = 0.0; // Default angle for group

                double ox, oy;
                if(sin(angle) > 0)
                    oy = cy - base;
                else
                    oy = cy + (point-base) * sin(angle) - base;

                if(cos(angle) >= 0)
                    ox = cx - base;
                else
                    ox = cx + (point-base) * cos(angle) - base;

                int dx, dy;
                costmap->worldToMapNoBounds(ox, oy, dx, dy);

                int start_x = 0, start_y = 0, end_x = width, end_y = height;
                if(dx < 0)
                    start_x = -dx;
                else if(dx + width > costmap->getSizeInCellsX())
                    end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

                if((int)(start_x+dx) < min_i)
                    start_x = min_i - dx;
                if((int)(end_x+dx) > max_i)
                    end_x = max_i - dx;

                if(dy < 0)
                    start_y = -dy;
                else if(dy + height > costmap->getSizeInCellsY())
                    end_y = std::max(0, (int)costmap->getSizeInCellsY() - dy);

                if((int)(start_y+dy) < min_j)
                    start_y = min_j - dy;
                if((int)(end_y+dy) > max_j)
                    end_y = max_j - dy;

                double bx = ox + res / 2, by = oy + res / 2;
                for(int i = start_x; i < end_x; i++) {
                    for(int j = start_y; j < end_y; j++) {
                        unsigned char old_cost = costmap->getCost(i+dx, j+dy);
                        if(old_cost == NO_INFORMATION)
                            continue;

                        double x = bx + i*res, y = by + j*res;
                        
                        // Check if point is within the group's circle
                        double dx_to_center = x - cx;
                        double dy_to_center = y - cy;
                        double distance_to_center = sqrt(dx_to_center*dx_to_center + dy_to_center*dy_to_center);
                        
                        // Skip if point is outside the group's circle
                        if (distance_to_center > effective_radius) {
                            continue;
                        }

                        // Calculate cost using the dynamically calculated tmp_covar
                        double cost = amplitude_ * exp(-(distance_to_center * distance_to_center) / (2 * tmp_covar));

                        if(cost < cutoff_)
                            continue;

                        unsigned char cvalue = (unsigned char)cost;
                        costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));
                    }
                }
            }
            catch(tf2::LookupException& ex) {
                ROS_ERROR("HumanGroupLayer: No Transform available Error: %s", ex.what());
                continue;
            }
            catch(tf2::ConnectivityException& ex) {
                ROS_ERROR("HumanGroupLayer: Connectivity Error: %s", ex.what());
                continue;
            }
            catch(tf2::ExtrapolationException& ex) {
                ROS_ERROR("HumanGroupLayer: Extrapolation Error: %s", ex.what());
                continue;
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::HumanGroupLayer, costmap_2d::Layer)
