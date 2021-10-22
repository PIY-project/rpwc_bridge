#include <request_from_web.h>

request_from_web::request_from_web()
{
    server_task_list_ = n_.advertiseService("/task_list_srv", &request_from_web::callback_task_list, this);

    path_pack_ = ros::package::getPath("rpwc");


}

request_from_web::~request_from_web()
{

}


//restituisce la lista dei task salvati, num_task array contenente il num di subtask per ogni task, subtask_names array contenente la lista degli  sub tasks
bool request_from_web::callback_task_list(rpwc_bridge::task_list::Request  &req, rpwc_bridge::task_list::Response &res)
{
    std::string path_obj_struct = path_pack_ + "/obj_structures";
    std::vector<std::string> task_list;
    task_list.clear();
    read_directory(path_obj_struct, task_list);
    for(int i = 0; i < task_list.size(); i++)
    {
        std::size_t found = task_list[i].find("_0");
        if (found!=std::string::npos)
        {
            std_msgs::String tmp_subtask_name;
            tmp_subtask_name.data.clear();
            tmp_subtask_name.data = task_list[i];
            tmp_subtask_name.data.erase (tmp_subtask_name.data.length()-5); //cancella .yaml 
            res.subtask_names.push_back(tmp_subtask_name);

            std::string path_yaml = path_pack_ + "/obj_structures/" + task_list[i];
            YAML::Node obj_yaml = YAML::LoadFile(path_yaml);
            std::string tmp_child;
            tmp_child.clear();
            tmp_child = obj_yaml["children"].as<std::string>();
            int num_subtasks = obj_yaml["num_tasks"].as<int>();
            std_msgs::Int32 tmp_num_subtask;
            tmp_num_subtask.data = num_subtasks;
            res.num_task.push_back(tmp_num_subtask);
            for(int j = 1; j < num_subtasks; j++)// j = 1 perchè il primo task è stato già listato
            {
                tmp_subtask_name.data.clear();
                tmp_subtask_name.data = tmp_child;
                // res.other_task_names.push_back(tmp_subtask_name);
                res.subtask_names.push_back(tmp_subtask_name);
                path_yaml.clear();
                path_yaml = path_pack_ + "/obj_structures/" + tmp_child + ".yaml";
                YAML::Node obj_yaml = YAML::LoadFile(path_yaml);
                tmp_child.clear();
                tmp_child = obj_yaml["children"].as<std::string>();
            }
        }
    }
    return true;
}


void request_from_web::read_directory(const std::string& name, std::vector<std::string>& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
    // sort( v.begin(), v.end() );
}