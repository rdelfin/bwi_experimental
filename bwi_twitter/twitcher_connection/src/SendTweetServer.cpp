#include "twitcher_connection/SendTweetServer.h"

#include "twitcher_connection/TwitterRequestHandler.h"
#include "twitcher_connection/TwitterApiCall.h"
#include "twitcher_connection/TwitterMentions.h"
#include "twitcher_connection/TwitterUpdateStatus.h"

SendTweetServer::SendTweetServer(std::string name, TwitterRequestHandler handler) :
    as_(nh_, name, boost::bind(&SendTweetServer::executeCB, this, _1), false),
    action_name_(name), handler(handler)
{
    ROS_INFO("Send Tweet action server starting");
    as_.start();
}



void SendTweetServer::executeCB(const twitcher_connection::SendTweetGoalConstPtr &goal)
{
    ros::Rate r(1);
    bool success = true;

    if(goal->mediaids.size() > 4) {
        ROS_WARN("Status contains more than 4 media ID's! Up to 4 are allowed");
    }
    
    feedback_.progress=0;
    as_.publishFeedback(feedback_);
    
    ROS_INFO("Sending tweet: \"%s\"", goal->message.c_str());
    
    feedback_.progress+=10;
    as_.publishFeedback(feedback_);

    TwitterApiCall* api = 
        new TwitterUpdateStatus(goal->message, -1, false, false, goal->mediaids);

    std::string resultString = handler.makeRequest(api);

    result_.success = success;

    ROS_INFO("Tweet sent");
    as_.setSucceeded(result_);

}
