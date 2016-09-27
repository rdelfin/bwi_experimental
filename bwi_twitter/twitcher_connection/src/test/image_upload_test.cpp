/*
 * Copyright 2015 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include <ros/ros.h>
#include <string>

#include <twitcher_connection/SendTweetAction.h>
#include <twitcher_connection/UploadMediaAction.h>

#include <actionlib/client/simple_action_client.h>

#include <json/json.hpp>

using json = nlohmann::json;



bool idConverterCallback(twitcher_connection::handle_from_id::Request&, twitcher_connection::handle_from_id::Response&);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_upload_test");
    ros::NodeHandle nh;
    
    actionlib::SimpleActionClient<>();
    
    ros::spin();
    return 0;
    
}

bool idConverterCallback(twitcher_connection::handle_from_id::Request& req, twitcher_connection::handle_from_id::Response& res)
{
    TwitterShowUser* showApi = new TwitterShowUser(req.id, "");
    std::string result = handler->makeRequest(showApi);
    delete showApi;
    
    if(result == "" || result == "[]") {
        ROS_WARN("Response from Show user API was empty!");
        res.handle = "";
        return false;
    }
    
    json root = json::parse(result);
    
    ROS_INFO_STREAM("Screen name result: " << root["screen_name"]);
    ROS_INFO_STREAM("Screen name lookup response: " << result);
    
    res.handle = root["screen_name"];
    
    return true;
}

