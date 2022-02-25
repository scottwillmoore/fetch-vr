using RosMessageTypes.Actionlib;
using System.Linq;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public enum GoalState
{
    Active,
    Done,
    Invalid,
    Pending,
    Preempting,
    Recalling,
    WaitingForCancelAcknowledgement,
    WaitingForGoalAcknowledgement,
    WaitingForResult,
}

public enum GoalTransition
{
    Aborted,
    Active,
    Lost,
    Pending,
    Preempted,
    Preempting,
    Recalled,
    Recalling,
    Rejected,
    Succeeded,
}

public enum ResultState
{
    Aborted,
    Lost,
    Preempted,
    Recalled,
    Rejected,
    Succeeded,
}


public class ROSActionClient<TGoal, TResult, TFeedback>
    where TGoal : Message, new()
    where TResult : Message, new()
    where TFeedback : Message, new()
{
    private readonly string actionName;

    private readonly string cancelPublisherName;
    private readonly string goalPublisherName;

    private readonly string feedbackSubscriberName;
    private readonly string resultSubscriberName;
    private readonly string statusSubscriberName;

    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private string goalId;

    private GoalState goalState;

    private TGoal latestGoal;
    private TResult latestResult;

    private Action<ResultState, TResult> doneAction;

    public ROSActionClient(string actionName)
    {
        this.actionName = actionName;

        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        cancelPublisherName = this.actionName + "/cancel";
        goalPublisherName = this.actionName + "/goal";

        feedbackSubscriberName = this.actionName + "/feedback";
        resultSubscriberName = this.actionName + "/result";
        statusSubscriberName = this.actionName + "/status";

        rosConnection.RegisterPublisher<GoalIDMsg>(cancelPublisherName);
        rosConnection.RegisterPublisher<ROSActionGoal<TGoal>>(goalPublisherName);

        rosConnection.Subscribe<ROSActionFeedback<TFeedback>>(feedbackSubscriberName, HandleFeedback);
        rosConnection.Subscribe<ROSActionResult<TResult>>(resultSubscriberName, HandleResult);
        rosConnection.Subscribe<GoalStatusArrayMsg>(statusSubscriberName, HandleStatus);

        goalState = GoalState.Done;
    }

    public void Send(TGoal goal, Action<ResultState, TResult> doneAction)
    {
        goalId = Guid.NewGuid().ToString();

        goalState = GoalState.WaitingForGoalAcknowledgement;

        latestGoal = goal;
        latestResult = null;

        this.doneAction = doneAction;

        var currentTime = rosTime.Now();
        ROSActionGoal<TGoal> actionGoal = new ROSActionGoal<TGoal>();
        actionGoal.goal = goal;
        actionGoal.goal_id.id = goalId;
        actionGoal.goal_id.stamp = currentTime;
        // actionGoal.header.frame_id = ...;
        actionGoal.header.stamp = currentTime;

        rosConnection.Publish(goalPublisherName, actionGoal);
    }

    private GoalStatusMsg FindStatus(GoalStatusMsg[] statuses)
    {
        return statuses.FirstOrDefault(status => status.goal_id.id == goalId);
    }

    private void HandleFeedback(ROSActionFeedback<TFeedback> actionFeedback)
    {
    }

    private void HandleResult(ROSActionResult<TResult> actionResult)
    {
        var status = FindStatus(new GoalStatusMsg[] { actionResult.status });
        if (status == null)
        {
            return;
        }

        var statusCode = status.status;
        var goalTransition = ParseTransition(statusCode);
        Transition(goalTransition);

        if (goalState == GoalState.Done)
        {
            throw new Exception("Invalid transition");
        }

        latestResult = actionResult.result;
        HandleTransition(goalState, goalTransition, GoalState.Done);
    }

    private void HandleStatus(GoalStatusArrayMsg goalStatusArray)
    {
        var status = FindStatus(goalStatusArray.status_list);
        if (status == null)
        {
            switch (goalState)
            {
                case GoalState.Active:
                case GoalState.Invalid:
                case GoalState.Pending:
                case GoalState.Preempting:
                case GoalState.Recalling:
                case GoalState.WaitingForCancelAcknowledgement:
                    Transition(GoalTransition.Lost);
                    break;

                case GoalState.Done:
                case GoalState.WaitingForGoalAcknowledgement:
                case GoalState.WaitingForResult:
                    break;
            }
            return;
        }

        var statusCode = status.status;
        var goalTransition = ParseTransition(statusCode);
        Transition(goalTransition);
    }

    private void HandleTransition(GoalState previousState, GoalTransition goalTransition, GoalState newState)
    {
        goalState = newState;

        switch (goalState)
        {
            case GoalState.Active:
                break;

            case GoalState.Done:
                var resultState = goalTransition switch
                {
                    GoalTransition.Aborted => ResultState.Aborted,
                    GoalTransition.Lost => ResultState.Lost,
                    GoalTransition.Preempted => ResultState.Preempted,
                    GoalTransition.Recalled => ResultState.Recalled,
                    GoalTransition.Rejected => ResultState.Rejected,
                    GoalTransition.Succeeded => ResultState.Succeeded,
                    _ => throw new Exception("Invalid transition")
                };
                if (doneAction != null)
                {
                    doneAction(resultState, latestResult);
                }
                break;

            case GoalState.Invalid:
                throw new Exception("Invalid transition");

            case GoalState.Pending:
                break;

            case GoalState.Preempting:
                break;

            case GoalState.Recalling:
                break;

            case GoalState.WaitingForCancelAcknowledgement:
                break;

            case GoalState.WaitingForGoalAcknowledgement:
                break;

            case GoalState.WaitingForResult:
                break;
        }
    }

    private GoalTransition ParseTransition(byte statusCode)
    {
        return statusCode switch
        {
            GoalStatusMsg.ABORTED => GoalTransition.Aborted,
            GoalStatusMsg.ACTIVE => GoalTransition.Active,
            GoalStatusMsg.LOST => GoalTransition.Lost,
            GoalStatusMsg.PENDING => GoalTransition.Pending,
            GoalStatusMsg.PREEMPTED => GoalTransition.Preempted,
            GoalStatusMsg.PREEMPTING => GoalTransition.Preempting,
            GoalStatusMsg.RECALLED => GoalTransition.Recalled,
            GoalStatusMsg.RECALLING => GoalTransition.Recalling,
            GoalStatusMsg.REJECTED => GoalTransition.Rejected,
            GoalStatusMsg.SUCCEEDED => GoalTransition.Succeeded,
            _ => throw new Exception("Invalid status"),
        };
    }

    private void Transition(GoalTransition goalTransition)
    {
        switch (goalState)
        {
            case GoalState.Done:
            case GoalState.Invalid:
                break;
        }

        switch (goalState, goalTransition)
        {
            // Active
            case (GoalState.Active, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Active, GoalTransition.Active):
                break;
            case (GoalState.Active, GoalTransition.Pending):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Active, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Active, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                break;
            case (GoalState.Active, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Active, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Active, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Active, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // Done
            case (GoalState.Done, _):
                break;

            // Lost
            case (_, GoalTransition.Lost):
                HandleTransition(goalState, goalTransition, GoalState.Done);
                break;

            // Pending
            case (GoalState.Pending, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Pending, GoalTransition.Active):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                break;
            case (GoalState.Pending, GoalTransition.Pending):
                break;
            case (GoalState.Pending, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Pending, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                break;
            case (GoalState.Pending, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.Recalling);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Pending, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Recalling);
                break;
            case (GoalState.Pending, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Pending, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // Preempting
            case (GoalState.Preempting, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Preempting, GoalTransition.Active):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Preempting, GoalTransition.Pending):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Preempting, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Preempting, GoalTransition.Preempting):
                break;
            case (GoalState.Preempting, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Preempting, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Preempting, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Preempting, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // Recalling
            case (GoalState.Recalling, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Recalling, GoalTransition.Active):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Recalling, GoalTransition.Pending):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.Recalling, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Recalling, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                break;
            case (GoalState.Recalling, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Recalling, GoalTransition.Recalling):
                break;
            case (GoalState.Recalling, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.Recalling, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // WaitingForCancelAcknowledgement
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Active):
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Pending):
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.Recalling);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Recalling);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForCancelAcknowledgement, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // WaitingForGoalAcknowledgement
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Aborted):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Active):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Pending):
                HandleTransition(goalState, goalTransition, GoalState.Pending);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Preempted):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.Preempting);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Recalled):
                HandleTransition(goalState, goalTransition, GoalState.Pending);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Pending);
                HandleTransition(goalState, goalTransition, GoalState.Recalling);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Rejected):
                HandleTransition(goalState, goalTransition, GoalState.Pending);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;
            case (GoalState.WaitingForGoalAcknowledgement, GoalTransition.Succeeded):
                HandleTransition(goalState, goalTransition, GoalState.Active);
                HandleTransition(goalState, goalTransition, GoalState.WaitingForResult);
                break;

            // WaitingForResult
            case (GoalState.WaitingForResult, GoalTransition.Aborted):
                break;
            case (GoalState.WaitingForResult, GoalTransition.Active):
                break;
            case (GoalState.WaitingForResult, GoalTransition.Pending):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.WaitingForResult, GoalTransition.Preempted):
                break;
            case (GoalState.WaitingForResult, GoalTransition.Preempting):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.WaitingForResult, GoalTransition.Recalled):
                break;
            case (GoalState.WaitingForResult, GoalTransition.Recalling):
                HandleTransition(goalState, goalTransition, GoalState.Invalid);
                break;
            case (GoalState.WaitingForResult, GoalTransition.Rejected):
                break;
            case (GoalState.WaitingForResult, GoalTransition.Succeeded):
                break;
        }
    }
}
