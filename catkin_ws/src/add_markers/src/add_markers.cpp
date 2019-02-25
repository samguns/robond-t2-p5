#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

enum states_e {
  picking_up = 0,
  picked,
  returning,
  drop_off,
  max_state
};

// Set our action
static uint32_t action = visualization_msgs::Marker::ADD;
// Set our target goal
static float x = 4.0;
static float y = 2.0;

static int counter = 0;

static states_e state = picking_up;

static void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  bool reached_goal(false);

  if ((fabs(msg->pose.pose.position.x - x) <= 0.2) &&
      (fabs(msg->pose.pose.position.y - y) <= 0.2)) {
    reached_goal = true;
  }

  switch (state) {
    case picking_up:
      if (reached_goal) {
        state = picked;
        action = visualization_msgs::Marker::DELETEALL;
      }

      break;

    case returning:
      if (reached_goal) {
        state = drop_off;
        action = visualization_msgs::Marker::ADD;
      }

      break;

    default:break;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber sub = n.subscribe("odom", 1000, odom_callback);

  // Set our shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  ros::Duration(6.0).sleep();

  while (ros::ok()) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
    ROS_INFO("Publish action %d to x:%f y:%f", action, x, y);

    ros::spinOnce();

    if (state == picked) {
      counter++;

      if (counter == 5) {
        state = returning;
        counter = 0;
        x = 1.0;
        y = 0.0;
      }
    }

    ros::Duration(1.0).sleep();
  }
}