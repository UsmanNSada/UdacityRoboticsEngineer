#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


/* class addmarkers to handle the publishing of markers at pickup and dropoff zones according
 to odometry values */
class AddMarkers{
private:
    // node handler, publisher and subscriber definitions
    ros::NodeHandle n_;
    ros::Publisher marker_pub;
    ros::Subscriber marker_sub;
    
public:
    // Class constructor
    AddMarkers(){
    //publisher, subscribers definitions and member function to publish marker at the pickup zone
    marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_sub = n_.subscribe("/odom", 100, &AddMarkers::OdomCallback, this);
    beginning(marker_pub);
    }
    // member function to publish the marker at the pickup zone at the beginning of the home-service session
    void beginning(ros::Publisher marker_pub){
        ros::Rate r(1);
        visualization_msgs::Marker marker;
        
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
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
                //return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        ROS_INFO("Connected to add_markers subscriber");

        //publish the marker for the pick up
        marker_pub.publish(marker);
        ROS_INFO("Published at pick_up");
    
    }

    // class Addmarkers member function for calculating distance between two x,y points
    float dist(float x1, float y1, float x2, float y2){
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
    }
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
       
        ros::Rate r(1);
        visualization_msgs::Marker marker;
        
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // Set the marker type. this is set to a CUBE
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        //ros::Duration(5).sleep();

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
        
        /*pickup is a variable that represents the distance between the robot pose and the item pickup zone.
         there are some inconsistencies in the robot pose values and the values of positions the markers -
         are published in the map. Thus, I determined the values -4.04, 4.22, 4.05, -5.47 as positions x,y and 
         x, y of the pickup and dropoff zones respectively according to the odometry values. 
        */ 
        float pickup = this->dist(msg->pose.pose.position.x, msg->pose.pose.position.y, 
                                -4.04, 4.22);

        //dropoff is a variable that represents the distance between the robot pose and marker dropoff zone
        volatile float dropoff = this->dist(msg->pose.pose.position.x, msg->pose.pose.position.y, 
                                  4.05, -5.47);
 
        // initializes the variable to identify wether the robot has reached the pickup zone
        bool Reachpickup = false;

        // this checks weather the robot is within 0.1 distance of the pickup zone
        if (pickup < 0.1){
            Reachpickup = true;
        }

        if(Reachpickup && dropoff > 0.1){
            //this hides the marker
            ros::Duration(5).sleep();
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);

            // another 5 seconds pause
            ros::Duration(5).sleep();
            
            // at this point the robot is traveling to the dropoff zone
            ROS_INFO("Traveling to dropoff zone");       
        }

        // Set the new pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = -5.63;
        marker.pose.position.y = -4.04;
        marker.pose.position.z = 0.50;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // this conditional checks wether the robot is within 0.1 of the dropoff zone and then drops it at the zone
        if(dropoff < 0.1) {
            //take the hidden marker and publish it
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Published at the drop-off");      
        }     
        
    }

};


int main( int argc, char** argv ){

    // Initialises the node and initialises AddMarkers class object to spin continously  
    ros::init(argc, argv, "add_markers");
    AddMarkers AddmarkerObject;
    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
    
}


 