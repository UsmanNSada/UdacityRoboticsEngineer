#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



int main( int argc, char** argv ){
    
    ros::init(argc, argv, "add_marker");

    ros::NodeHandle n;
   
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 
    
    visualization_msgs::Marker marker;
        
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_marker";
    marker.id = 0;

    // Set the marker type. this is set to a CUBE
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 4.31;
    marker.pose.position.y = 4.18;
    marker.pose.position.z = 0.50;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.40;
    marker.scale.y = 0.40;
    marker.scale.z = 0.40;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    ROS_INFO("Connected to add_markers subscriber");

    //publish the marker for the pick up
    marker_pub.publish(marker);
    ROS_INFO("Published at pick_up");

    ros::Duration(5).sleep();
    marker.action = visualization_msgs::Marker::DELETE;

    marker_pub.publish(marker);
    // another 5 seconds pause
    ros::Duration(5).sleep();


    // Set the new pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -5.63;
    marker.pose.position.y = -4.04;
    marker.pose.position.z = 0.50;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.00;

    //take the hidden marker and publish it
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ROS_INFO("Published at the drop-off");      
  
}


 