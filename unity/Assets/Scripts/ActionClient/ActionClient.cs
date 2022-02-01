using RosMessageTypes.Actionlib;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector;

public class ActionClient<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback>
    where TActionGoal : ActionGoal<TGoal>, new()
    where TActionResult : ActionResult<TResult>, new()
    where TActionFeedback : ActionFeedback<TFeedback>, new()
    where TGoal : Message, new()
    where TResult : Message, new()
    where TFeedback : Message, new()
{
    public enum State
    {
        Active,
        Done,
        Pending,
        Preempting,
        Recalling,
        WaitingForCancelAcknowledgement,
        WaitingForGoalAcknowledgement,
        WaitingForResult,
    }

    public enum Status
    {
        Aborted,
        Active,
        Pending,
        Preempted,
        Preempting,
        Recalled,
        Recalling,
        Rejected,
        Succeeded,
    }

    private static Status DecodeStatus(byte encodedStatus)
    {
        return encodedStatus switch
        {
            GoalStatusMsg.ABORTED => Status.Aborted,
            GoalStatusMsg.ACTIVE => Status.Active,
            GoalStatusMsg.PENDING => Status.Pending,
            GoalStatusMsg.PREEMPTED => Status.Preempted,
            GoalStatusMsg.PREEMPTING => Status.Preempting,
            GoalStatusMsg.RECALLED => Status.Recalled,
            GoalStatusMsg.RECALLING => Status.Recalling,
            GoalStatusMsg.REJECTED => Status.Rejected,
            GoalStatusMsg.SUCCEEDED => Status.Succeeded,
            _ => throw new ActionClientException(),
        };
    }

    private static byte EncodeStatus(Status status)
    {
        return status switch
        {
            Status.Aborted => GoalStatusMsg.ABORTED,
            Status.Active => GoalStatusMsg.ACTIVE,
            Status.Pending => GoalStatusMsg.PENDING,
            Status.Preempted => GoalStatusMsg.PREEMPTED,
            Status.Preempting => GoalStatusMsg.PREEMPTING,
            Status.Recalled => GoalStatusMsg.RECALLED,
            Status.Recalling => GoalStatusMsg.RECALLING,
            Status.Rejected => GoalStatusMsg.REJECTED,
            Status.Succeeded => GoalStatusMsg.SUCCEEDED,
            _ => throw new ActionClientException(),
        };
    }

    private readonly string actionName;

    private readonly string cancelPublisherName;
    private readonly string goalPublisherName;

    private readonly string feedbackSubscriberName;
    private readonly string resultSubscriberName;
    private readonly string statusSubscriberName;

    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private State state;

    private string goalId;

    private Action<TFeedback> feedbackCallback;
    private Action<TResult> doneCallback;

    public ActionClient(string actionName)
    {
        this.actionName = actionName;

        cancelPublisherName = this.actionName + "/cancel";
        goalPublisherName = this.actionName + "/goal";

        feedbackSubscriberName = this.actionName + "/feedback";
        resultSubscriberName = this.actionName + "/result";
        statusSubscriberName = this.actionName + "/status";

        // this.feedbackCallback = feedbackCallback;
        // this.transitionCallback = transitionCallback;

        state = State.Done;

        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        rosConnection.RegisterPublisher<GoalIDMsg>(cancelPublisherName);
        rosConnection.RegisterPublisher<TActionGoal>(goalPublisherName);

        rosConnection.Subscribe<GoalStatusArrayMsg>(statusSubscriberName, StatusCallback);
        rosConnection.Subscribe<TActionFeedback>(feedbackSubscriberName, FeedbackCallback);
        rosConnection.Subscribe<TActionResult>(resultSubscriberName, ResultCallback);
    }

    public void Cancel()
    {
        if (goalId == null)
        {
            throw new InvalidOperationException("Goal has not been set!");
        }

        // TODO!
        // rosConnection.Publish(cancelPublisherName, )
    }

    public void SendGoal(TGoal goal, Action<TResult> doneCallback = null, Action<TFeedback> feedbackCallback = null)
    {
        if (goal == null)
        {
            throw new ArgumentNullException(nameof(goal));
        }

        // TODO: What if the goal exists...

        state = State.WaitingForGoalAcknowledgement;

        goalId = GenerateId();

        this.doneCallback = doneCallback;
        this.feedbackCallback = feedbackCallback;

        var currentTime = rosTime.Now();

        TActionGoal actionGoal = new TActionGoal();
        actionGoal.goal = goal;
        actionGoal.goal_id.id = goalId;
        actionGoal.goal_id.stamp = currentTime;
        // actionGoal.header.frame_id = ...;
        actionGoal.header.stamp = currentTime;

        rosConnection.Publish(goalPublisherName, actionGoal);
    }

    private void FeedbackCallback(TActionFeedback actionFeedback)
    {
        var feedbackGoalId = actionFeedback.status.goal_id.id;
        if (feedbackGoalId != goalId)
        {
            return;
        }

        var encodedStatus = actionFeedback.status.status;
        var status = DecodeStatus(encodedStatus);
        UpdateState(status);

        var feedback = actionFeedback.feedback;

        // TODO!
    }

    private string GenerateId()
    {
        return Guid.NewGuid().ToString();
    }

    private void InvalidState()
    {
        throw new ActionClientException();
    }

    private void ResultCallback(TActionResult actionResult)
    {
        var resultGoalId = actionResult.status.goal_id.id;
        if (resultGoalId != goalId)
        {
            return;
        }

        var encodedStatus = actionResult.status.status;
        var status = DecodeStatus(encodedStatus);
        UpdateState(status);

        var result = actionResult.result;

        // TODO!
    }

    private void SetState(State newState)
    {
        state = newState;
    }

    private void StatusCallback(GoalStatusArrayMsg goalStatusArrayMessage)
    {
        var goalStatusArray = goalStatusArrayMessage.status_list;
        foreach (var goalStatus in goalStatusArray)
        {
            var statusGoalId = goalStatus.goal_id.id;
            if (statusGoalId != goalId)
            {
                continue;
            }

            var encodedStatus = goalStatus.status;
            var status = DecodeStatus(encodedStatus);
            UpdateState(status);
        }
    }

    private void UpdateState(Status status)
    {
        if (state == State.Done)
        {
            return;
        }

        switch (state)
        {
            case State.Active:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        break;
                    case Status.Pending:
                        InvalidState();
                        break;
                    case Status.Preempted:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        SetState(State.Preempting);
                        break;
                    case Status.Recalled:
                        InvalidState();
                        break;
                    case Status.Recalling:
                        InvalidState();
                        break;
                    case Status.Rejected:
                        InvalidState();
                        break;
                    case Status.Succeeded:
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.Done:
                switch (status)
                {
                    case Status.Aborted:
                        break;
                    case Status.Active:
                        InvalidState();
                        break;
                    case Status.Pending:
                        InvalidState();
                        break;
                    case Status.Preempted:
                        break;
                    case Status.Preempting:
                        InvalidState();
                        break;
                    case Status.Recalled:
                        break;
                    case Status.Recalling:
                        InvalidState();
                        break;
                    case Status.Rejected:
                        break;
                    case Status.Succeeded:
                        break;
                }
                break;

            case State.Pending:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.Active);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        SetState(State.Active);
                        break;
                    case Status.Pending:
                        break;
                    case Status.Preempted:
                        SetState(State.Active);
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        SetState(State.Active);
                        SetState(State.Preempting);
                        break;
                    case Status.Recalled:
                        SetState(State.Recalling);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Recalling:
                        SetState(State.Recalling);
                        break;
                    case Status.Rejected:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Succeeded:
                        SetState(State.Active);
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.Preempting:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        InvalidState();
                        break;
                    case Status.Pending:
                        InvalidState();
                        break;
                    case Status.Preempted:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        break;
                    case Status.Recalled:
                        InvalidState();
                        break;
                    case Status.Recalling:
                        InvalidState();
                        break;
                    case Status.Rejected:
                        InvalidState();
                        break;
                    case Status.Succeeded:
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.Recalling:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        InvalidState();
                        break;
                    case Status.Pending:
                        InvalidState();
                        break;
                    case Status.Preempted:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        SetState(State.Preempting);
                        break;
                    case Status.Recalled:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Recalling:
                        break;
                    case Status.Rejected:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Succeeded:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.WaitingForCancelAcknowledgement:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        break;
                    case Status.Pending:
                        break;
                    case Status.Preempted:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        SetState(State.Preempting);
                        break;
                    case Status.Recalled:
                        SetState(State.Recalling);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Recalling:
                        SetState(State.Recalling);
                        break;
                    case Status.Rejected:
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Succeeded:
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.WaitingForGoalAcknowledgement:
                switch (status)
                {
                    case Status.Aborted:
                        SetState(State.Active);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Active:
                        SetState(State.Active);
                        break;
                    case Status.Pending:
                        SetState(State.Pending);
                        break;
                    case Status.Preempted:
                        SetState(State.Active);
                        SetState(State.Preempting);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Preempting:
                        SetState(State.Active);
                        SetState(State.Preempting);
                        break;
                    case Status.Recalled:
                        SetState(State.Pending);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Recalling:
                        SetState(State.Pending);
                        SetState(State.Recalling);
                        break;
                    case Status.Rejected:
                        SetState(State.Pending);
                        SetState(State.WaitingForResult);
                        break;
                    case Status.Succeeded:
                        SetState(State.Active);
                        SetState(State.WaitingForResult);
                        break;
                }
                break;

            case State.WaitingForResult:
                switch (status)
                {
                    case Status.Aborted:
                        InvalidState();
                        break;
                    case Status.Active:
                        break;
                    case Status.Pending:
                        InvalidState();
                        break;
                    case Status.Preempted:
                        break;
                    case Status.Preempting:
                        InvalidState();
                        break;
                    case Status.Recalled:
                        break;
                    case Status.Recalling:
                        InvalidState();
                        break;
                    case Status.Rejected:
                        break;
                    case Status.Succeeded:
                        break;
                }
                break;
        }
    }
}

