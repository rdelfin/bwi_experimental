/*
 * Copyright 2016 <copyright holder> <email>
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

#include "twitcher_connection/MediaUploadServer.h"
#include "twitcher_connection/TwitterUploadMedia.h"
#include "twitcher_connection/base64.h"
#include "json/json.hpp"

using json = nlohmann::json;

MediaUploadServer::MediaUploadServer(std::string name, TwitterRequestHandler handler)
    : as_(nh_, name, boost::bind(&MediaUploadServer::executeCB, this, _1), false),
      action_name_(name), handler(handler)
{
    ROS_INFO("Media Upload action server starting");
    as_.start();
}
  
void MediaUploadServer::executeCB(const twitcher_connection::UploadMediaGoalConstPtr &goal)
{
    ros::Rate r(1);
    bool success = true;

    feedback_.progress=0;

    ROS_INFO("Uploading image of size: %d,%d", goal->media.width, goal->media.height);

    feedback_.progress+=10;

    TwitterApiCall* api = 
        new TwitterUploadMedia(base64_encode(&goal->media.data[0], goal->media.data.size()));
    
    std::string resultString = handler.makeRequest(api);
    
    json resultJson = json::parse(resultString);
    
    result_.success = success;
    result_.media_id = resultJson["media_id"];

    ROS_INFO("Tweet sent");
    as_.setSucceeded(result_);

}