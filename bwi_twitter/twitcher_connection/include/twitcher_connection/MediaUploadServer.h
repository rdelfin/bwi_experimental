/*
 * Copyright 2016 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
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



#include <actionlib/server/simple_action_server.h>
#include "twitcher_connection/UploadMediaAction.h"
#include "twitcher_connection/TwitterRequestHandler.h"

#ifndef MEDIAUPLOADSERVER_H
#define MEDIAUPLOADSERVER_H

class MediaUploadServer
{
public:
    MediaUploadServer(std::string name, TwitterRequestHandler handler);
  
    void executeCB(const twitcher_connection::UploadMediaGoalConstPtr &goal);

    ~MediaUploadServer(void) { }
  
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<twitcher_connection::UploadMediaAction> as_;
    std::string action_name_;

    twitcher_connection::UploadMediaFeedback feedback_;
    twitcher_connection::UploadMediaResult result_;
    TwitterRequestHandler& handler;
};

#endif // MEDIAUPLOADSERVER_H
