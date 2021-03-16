#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "Poco/Process.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <rpwc_bridge/setup_hardware.h>
#include <rpwc_bridge/set_jog_mode_web.h>


Poco::ProcessHandle* ph;
Poco::ProcessHandle* ph_jog_mode_;
std::string last_jog_mode = "none";


bool running = false;
bool running_arm_ = false;
bool running_ee_ = false;
bool running_cam_ = false;

std::vector<Poco::ProcessHandle *> ph_vec_;
std::vector<std::string> names_vec_;

void run_launch(std::string cmd_line)
{

}

void callback_run(std_msgs::String msg){
    if(!running){
        std::vector<std::string> args;
        boost::split(args, msg.data, boost::is_any_of(" ") ); //Split the msg.data on space and save it to a vector
        Poco::ProcessHandle ph_running = Poco::Process::launch("roslaunch", args,0,0,0); //launch a new node
        ph = new Poco::ProcessHandle(ph_running); // Copy the processhandler to our global variable
        running = true; //Refuse new launch
        ROS_INFO_STREAM("launched : roslaunch " << msg.data);
    }
    else{
        ROS_ERROR("A process is already running.");
    }
}

void callback_kill(std_msgs::Empty msg){
    if(running){
        Poco::Process::requestTermination(ph->id()); //send SIGINT
            Poco::Process::wait(*ph); //Wait for roslaunch to kill every node
        free(ph);  
        running = false;  //accept a new launch
        ROS_INFO("Killed process");
    }
    else{
        ROS_ERROR("No Process are running.");
    }
}

bool callback_hw_launch(rpwc_bridge::setup_hardware::Request  &req, rpwc_bridge::setup_hardware::Response &res)
{   
    std::cout << "sono in callback_hw_launch"<< std::endl;
    bool active = req.active.data;
    std::cout << "bool active: "<< active <<std::endl;

    bool is_running;
    int process_index;
    std::string name;
    name = req.brand.data + req.model.data + req.ip.data;
    for(int i = 0; i < names_vec_.size(); i++)
    {
        if(names_vec_[i].compare(name) == 0)
        {
            is_running = true;
            process_index = i;
            break;
        }
        else is_running = false;
    }


    if(active)
    {
        // req.ns
        // req.brand
        // req.model
        // req.ip
        
        if(!is_running)
        {
            names_vec_.push_back(name);
            std::string cmd_line;
            // cmd_line = "rpwc_" + req.brand.data + "_launch" + " " + req.model.data + ".launch" + " " + "ns_setup:=" + "\"" + req.ns.data + "\"" + " " + "ip:=" + "\"" + req.ip.data + "\"";
            cmd_line = "rpwc_" + req.brand.data + "_launch" + " " + req.model.data + ".launch" + " " + "ns_setup:="  + req.ns.data  + " " + "ip:="  + req.ip.data ;
            std::vector<std::string> args;
            boost::split(args, cmd_line, boost::is_any_of(" ") ); //Split the msg.data on space and save it to a vector
            Poco::ProcessHandle ph_running = Poco::Process::launch("roslaunch", args,0,0,0); //launch a new node

            ph_vec_.push_back(new Poco::ProcessHandle(ph_running)); // Copy the processhandler to our global variable
            ROS_INFO_STREAM("launched : roslaunch " << cmd_line);
        }
        else ROS_INFO_STREAM("the process is already running");

    }
    else
    {
        if(is_running)
        {
            Poco::Process::requestTermination(ph_vec_[process_index]->id()); //send SIGINT
            Poco::Process::wait(*ph_vec_[process_index]); //Wait for roslaunch to kill every node
            free(ph_vec_[process_index]);
            ph_vec_.erase (ph_vec_.begin() + process_index);
            names_vec_.erase (names_vec_.begin() + process_index);
            ROS_INFO("Killed process");
        }
        else ROS_ERROR("No Process are running.");

    }

    res.answer.data = true;
    return true;
}

bool callback_jog_launch(rpwc_bridge::set_jog_mode_web::Request  &req, rpwc_bridge::set_jog_mode_web::Response &res)
{

    std::string jog_mode = req.jog_mode_name.data;

    if(jog_mode.compare(last_jog_mode) != 0)
    {
        std::string cmd_line;
        if(jog_mode.compare("teleop") == 0)
        {
            cmd_line = "oculus_robot_bridge rpwc_oculus_bridge.launch ns_setup:="  + req.ns.data;
            std::vector<std::string> args;
            boost::split(args, cmd_line, boost::is_any_of(" ") ); //Split the msg.data on space and save it to a vector
            Poco::ProcessHandle ph_running = Poco::Process::launch("roslaunch", args,0,0,0); //launch a new node
            ph_jog_mode_ = new Poco::ProcessHandle(ph_running); // Copy the processhandler to our global variable
        }
        else if(jog_mode.compare("smart") == 0)
        {
            // cmd_line = "---------------------------------------.launch ns_setup:="  + req.ns.data;
            // std::vector<std::string> args;
            // boost::split(args, cmd_line, boost::is_any_of(" ") ); //Split the msg.data on space and save it to a vector
            // Poco::ProcessHandle ph_running = Poco::Process::launch("roslaunch", args,0,0,0); //launch a new node
            // ph_jog_mode_ = new Poco::ProcessHandle(ph_running); // Copy the processhandler to our global variable
        }
        else if(jog_mode.compare("gravity") == 0)
        {
            if(last_jog_mode.compare("none") != 0)
            {
                // kill process ph_jog_mode_
                Poco::Process::requestTermination(ph_jog_mode_->id()); //send SIGINT
                Poco::Process::wait(*ph_jog_mode_); //Wait for roslaunch to kill every node
                free(ph_jog_mode_); 
            }
            // call service to switch_ctr_from_web
        }
        else if(jog_mode.compare("none") == 0)
        {
            if(last_jog_mode.compare("gravity") != 0)
            {
                // kill process ph_jog_mode_
                Poco::Process::requestTermination(ph_jog_mode_->id()); //send SIGINT
                Poco::Process::wait(*ph_jog_mode_); //Wait for roslaunch to kill every node
                free(ph_jog_mode_); 
            }
            else
            {
                // call service to switch_ctr_from_web in position control
            }
            
        }
    }

    last_jog_mode = jog_mode;
    
    return true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "node_runner");
    ros::NodeHandle n;
    ros::Rate r(30.0);
    ros::Subscriber sub_run = n.subscribe("/run",100,callback_run);
    ros::Subscriber sub_kill = n.subscribe("/kill",100,callback_kill);

    ros::ServiceServer service_hw = n.advertiseService("/hw_launch", callback_hw_launch);
    ros::ServiceServer service_jog = n.advertiseService("/jog_launch", callback_jog_launch);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }// end while()
    return 0;
}