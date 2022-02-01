// http://wiki.ros.org/actionlib/DetailedDescription
// https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/Actionlib/ActionClient.cs
// https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_client.py
// https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/action_client.py

// ROSAction
// ROSActionClient
// ROSActionServer
// ROSConnection
// ROSFeedback
// ROSGoal
// ROSMessage
// ROSNode
// ROSService
// ROSTime
// ROSTopic

using RosMessageTypes.Actionlib;
using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

using ROS = Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class SimpleActionClient<TAction, TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TAction : ROS.Action<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
        where TActionGoal : ROS.ActionGoal<TGoal>, new()
        where TActionResult : ROS.ActionResult<TResult>
        where TActionFeedback : ROS.ActionFeedback<TFeedback>
        where TGoal : ROS.Message
        where TResult : ROS.Message
        where TFeedback : ROS.Message
{
    private readonly string actionName;

    private readonly string cancelPublisherName;
    private readonly string goalPublisherName;

    private readonly string feedbackSubscriberName;
    private readonly string resultSubscriberName;
    private readonly string statusSubscriberName;

    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private Action<TResult> doneCallback;
    private Action<TFeedback> feedbackCallback;

    private string goalId;

    public SimpleActionClient(string actionName)
    {
        this.actionName = actionName;

        cancelPublisherName = $"{this.actionName}/cancel";
        goalPublisherName = $"{this.actionName}/goal";

        feedbackSubscriberName = $"{this.actionName}/feedback";
        resultSubscriberName = $"{this.actionName}/result";
        statusSubscriberName = $"{this.actionName}/status";

        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        rosConnection.RegisterPublisher<GoalIDMsg>(cancelPublisherName);
        rosConnection.RegisterPublisher<TActionGoal>(goalPublisherName);

        rosConnection.Subscribe<GoalStatusArrayMsg>(statusSubscriberName, StatusCallback);
        rosConnection.Subscribe<TActionFeedback>(feedbackSubscriberName, FeedbackCallback);
        rosConnection.Subscribe<TActionResult>(resultSubscriberName, ResultCallback);
    }

    public void Send(TGoal goal, Action<TResult> callback)
    {
        doneCallback = callback;

        goalId = Guid.NewGuid().ToString();

        var latestTime = rosTime.Now();

        TActionGoal actionGoal = new TActionGoal();
        actionGoal.goal = goal;
        actionGoal.goal_id.id = goalId;
        actionGoal.goal_id.stamp = latestTime;
        // actionGoal.header.frame_id = ...;
        actionGoal.header.stamp = latestTime;

        rosConnection.Publish(goalPublisherName, actionGoal);
    }

    private void FeedbackCallback(TActionFeedback actionFeedback)
    {
        var resultGoalId = actionFeedback.status.goal_id.id;
        if (resultGoalId != goalId)
            return;
    }

    private void ResultCallback(TActionResult actionResult)
    {
        var resultGoalId = actionResult.status.goal_id.id;
        Debug.Log("resultGoalId: " + resultGoalId + " goalId: " + goalId);
        if (resultGoalId != goalId)
            return;

        var goalStatus = actionResult.status.status;
        switch (goalStatus)
        {
            case GoalStatusMsg.ABORTED:
            case GoalStatusMsg.ACTIVE:
            case GoalStatusMsg.LOST:
            case GoalStatusMsg.PENDING:
            case GoalStatusMsg.PREEMPTED:
            case GoalStatusMsg.RECALLED:
            case GoalStatusMsg.RECALLING:
            case GoalStatusMsg.REJECTED:
            case GoalStatusMsg.SUCCEEDED:
                Debug.Log("goalStatus: " + goalStatus);

                if (doneCallback != null)
                {
                    doneCallback(actionResult.result);
                }

                break;

            default:
                break;
        }
    }

    private void StatusCallback(GoalStatusArrayMsg goalStatusArray)
    {
        var statusArray = goalStatusArray.status_list;
        foreach (var status in statusArray)
        {
            var resultGoalId = status.goal_id.id;

            if (resultGoalId != goalId)
                continue;

            var statusCode = status.status;
            var statusMessage = status.text;

            switch (statusCode)
            {
                case GoalStatusMsg.ABORTED:

                default:
                    break;
            }
        }
    }
}

