
#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

float location[3]={0,0,2}; //the destination/mission to be published
float wpset[7][2] = {{0,0},{10,0},{5,-sqrt(75)},{-5,-sqrt(75)},{-10,0},{-5,sqrt(75)},{5,sqrt(75)}}; //array of wp insertion to location
int mission = 0; //counter wp/mission


//checking the position right now
geometry_msgs::PoseStamped posenow;
int stabilize(int x, int y, int z){
    float miss=0.1;
    if (posenow.pose.position.x>location[x]-miss&&posenow.pose.position.x<location[x]+miss&&
        posenow.pose.position.y>location[y]-miss&&posenow.pose.position.y<location[y]+miss&&
        posenow.pose.position.z>location[z]-miss&&posenow.pose.position.z<location[z]+miss)
        return 1;
    return -1;
}

//callback wp set
geometry_msgs::PoseStamped beforePos;
void pos_cb(geometry_msgs::PoseStamped posSent){  
    beforePos.pose.position.x=posSent.pose.position.x;
    beforePos.pose.position.y=posSent.pose.position.y;
    beforePos.pose.position.z=posSent.pose.position.z;
}


//callback current state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // FILE* pipe = popen("python example.py", {"-r"});
    // if (!pipe) {
    //     fprintf(stderr, "Failed to run Python script\n");
    //     return 1;
    // }
    // pclose(pipe);
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //set the first position as takeoff coordinates
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = location[0]; //mean x=0
    pose.pose.position.y = location[1]; //mean y=0 
    pose.pose.position.z = location[2]; //mean z=0

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //custom offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //custom arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //while ros is on and everything's ok (maybe except the programmer)
    while(ros::ok()){
        // if the mode wasn't offboard, set offboard
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        // if uav wasn't armed then armed please 
        else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            
        } 
        //if uav armed and offboard-ed then do mission following wpo\ or coordinates command
        else{
            // idk why i write "pose.pose.position.z == 2" besides there's checker function "stabilize"
            // just like --> affraid of error again + tired = not erase it. sorry sir
            if (pose.pose.position.z == 2&&stabilize(0,1,2)&&mission<7&&
                (ros::Time::now() - last_request > ros::Duration(20.0))) {
                //insert wp mission to location
                location[0]+=wpset[mission][0];
                location[1]+=wpset[mission][1];
                pose.pose.position.x=location[0];
                pose.pose.position.y=location[1];
                for(int i = 100; ros::ok() && i > 0; --i){
                    local_pos_pub.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                }
                //count mission
                mission++;
                ROS_INFO("Running Mission %d", mission);
            } 
            else if (mission==7){
                ROS_INFO("All Mission Completed");
                // i choose to end this here, actually i want to land it, i've tried but yeah fail and exhausted
                return 0;
            }
        }


        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
//that's all from me, thanks for reading the code till this point. oh yeah, i know there aren't an ideal flight at all.
}
