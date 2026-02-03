#include <deque>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Geometry>

namespace {

template <typename T>
T clamp(const T& v, const T& lo, const T& hi) {
  return std::max(lo, std::min(hi, v));
}

Eigen::Quaterniond toEigenQ(const geometry_msgs::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

geometry_msgs::Quaternion toMsgQ(const Eigen::Quaterniond& q)
{
  geometry_msgs::Quaternion m;
  Eigen::Quaterniond nq = q.normalized();
  m.x = nq.x();
  m.y = nq.y();
  m.z = nq.z();
  m.w = nq.w();
  return m;
}

class OdomInterpolator {
public:
  OdomInterpolator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, std::string("/vins_fusion/odometry"));
    pnh.param<std::string>("output_topic", output_topic_, std::string("/vins_fusion/odometry_200hz"));
    pnh.param("target_rate", target_rate_, 200.0);
    pnh.param("max_extrapolation", max_extrapolation_sec_, 0.05);

    pub_ = nh.advertise<nav_msgs::Odometry>(output_topic_, 200);
    sub_ = nh.subscribe(input_topic_, 10, &OdomInterpolator::odomCb, this);

    double period = 1.0 / std::max(1.0, target_rate_);
    timer_ = nh.createTimer(ros::Duration(period), &OdomInterpolator::onTimer, this);

    ROS_INFO_STREAM("odom_interpolator (cpp): input=" << input_topic_
                    << ", output=" << output_topic_ \
                    << ", rate=" << target_rate_ << " Hz");
  }

private:
  void odomCb(const nav_msgs::OdometryConstPtr& msg)
  {
    if (buffer_.size() == 2) buffer_.pop_front();
    buffer_.push_back(msg);
  }

  void publishClone(const nav_msgs::Odometry& src, const ros::Time& stamp)
  {
    nav_msgs::Odometry out = src;
    out.header.stamp = stamp;
    pub_.publish(out);
  }

  void publishInterp(const nav_msgs::Odometry& m0, const nav_msgs::Odometry& m1, double alpha, const ros::Time& stamp)
  {
    nav_msgs::Odometry out;
    out.header.frame_id = m1.header.frame_id;
    out.child_frame_id = m1.child_frame_id;
    out.header.stamp = stamp;

    // Position
    out.pose.pose.position.x = m0.pose.pose.position.x + alpha * (m1.pose.pose.position.x - m0.pose.pose.position.x);
    out.pose.pose.position.y = m0.pose.pose.position.y + alpha * (m1.pose.pose.position.y - m0.pose.pose.position.y);
    out.pose.pose.position.z = m0.pose.pose.position.z + alpha * (m1.pose.pose.position.z - m0.pose.pose.position.z);

    // Orientation (slerp)
    Eigen::Quaterniond q0 = toEigenQ(m0.pose.pose.orientation);
    Eigen::Quaterniond q1 = toEigenQ(m1.pose.pose.orientation);
    Eigen::Quaterniond qi = q0.slerp(alpha, q1);
    out.pose.pose.orientation = toMsgQ(qi);

    // Twist (linear interp)
    out.twist.twist.linear.x = (1 - alpha) * m0.twist.twist.linear.x + alpha * m1.twist.twist.linear.x;
    out.twist.twist.linear.y = (1 - alpha) * m0.twist.twist.linear.y + alpha * m1.twist.twist.linear.y;
    out.twist.twist.linear.z = (1 - alpha) * m0.twist.twist.linear.z + alpha * m1.twist.twist.linear.z;
    out.twist.twist.angular.x = (1 - alpha) * m0.twist.twist.angular.x + alpha * m1.twist.twist.angular.x;
    out.twist.twist.angular.y = (1 - alpha) * m0.twist.twist.angular.y + alpha * m1.twist.twist.angular.y;
    out.twist.twist.angular.z = (1 - alpha) * m0.twist.twist.angular.z + alpha * m1.twist.twist.angular.z;

    // Covariances: take from newer
    out.pose.covariance = m1.pose.covariance;
    out.twist.covariance = m1.twist.covariance;

    pub_.publish(out);
  }

  void publishExtrapolate(const nav_msgs::Odometry& m, double dt, const ros::Time& stamp)
  {
    nav_msgs::Odometry out;
    out.header.frame_id = m.header.frame_id;
    out.child_frame_id = m.child_frame_id;
    out.header.stamp = stamp;

    // Hold orientation, constant-velocity position extrapolation
    out.pose.pose.orientation = m.pose.pose.orientation;
    out.pose.pose.position.x = m.pose.pose.position.x + m.twist.twist.linear.x * dt;
    out.pose.pose.position.y = m.pose.pose.position.y + m.twist.twist.linear.y * dt;
    out.pose.pose.position.z = m.pose.pose.position.z + m.twist.twist.linear.z * dt;

    out.twist = m.twist;
    out.pose.covariance = m.pose.covariance;
    out.twist.covariance = m.twist.covariance;

    pub_.publish(out);
  }

  void onTimer(const ros::TimerEvent&)
  {
    if (buffer_.empty()) return;

    const ros::Time now = ros::Time::now();

    if (buffer_.size() == 1) {
      publishClone(*buffer_.back(), now);
      return;
    }

    const nav_msgs::Odometry& m0 = *buffer_[0];
    const nav_msgs::Odometry& m1 = *buffer_[1];

    const double t0 = m0.header.stamp.toSec();
    const double t1 = m1.header.stamp.toSec();

    if (t1 <= t0) {
      publishClone(m1, now);
      return;
    }

    const double t = now.toSec();
    if (t <= t0) {
      publishInterp(m0, m1, 0.0, now);
    } else if (t >= t1) {
      const double dt = t - t1;
      if (dt <= max_extrapolation_sec_) {
        publishExtrapolate(m1, dt, now);
      } else {
        publishClone(m1, now);
      }
    } else {
      const double alpha = clamp((t - t0) / (t1 - t0), 0.0, 1.0);
      publishInterp(m0, m1, alpha, now);
    }
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  double target_rate_ {200.0};
  double max_extrapolation_sec_ {0.05};

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;

  std::deque<nav_msgs::OdometryConstPtr> buffer_;
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_interpolator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OdomInterpolator node(nh, pnh);
  ros::spin();
  return 0;
}
