using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Moveit;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public static class ROSExtensions
{
    private static readonly double secondsPerNanosecond = 1e-9;

#if UNITY_EDITOR
    [UnityEditor.InitializeOnLoadMethod]
#else
    [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
    public static void Register()
    {
        MessageRegistry.Register(ROSActionGoal<ExecuteTrajectoryGoal>.k_RosMessageName, ROSActionGoal<ExecuteTrajectoryGoal>.Deserialize);
        MessageRegistry.Register(ROSActionResult<ExecuteTrajectoryResult>.k_RosMessageName, ROSActionResult<ExecuteTrajectoryResult>.Deserialize);
        MessageRegistry.Register(ROSActionFeedback<ExecuteTrajectoryFeedback>.k_RosMessageName, ROSActionFeedback<ExecuteTrajectoryFeedback>.Deserialize);

        MessageRegistry.Register(ROSActionGoal<MoveGroupGoal>.k_RosMessageName, ROSActionGoal<MoveGroupGoal>.Deserialize);
        MessageRegistry.Register(ROSActionResult<MoveGroupResult>.k_RosMessageName, ROSActionResult<MoveGroupResult>.Deserialize);
        MessageRegistry.Register(ROSActionFeedback<MoveGroupFeedback>.k_RosMessageName, ROSActionFeedback<MoveGroupFeedback>.Deserialize);
    }

    public static double ToDouble(this DurationMsg durationMessage)
    {
        var duration = (double)durationMessage.sec;
        duration += secondsPerNanosecond * (double)durationMessage.nanosec;
        return duration;
    }

    public static string GetErrorMessage(this MoveItErrorCodesMsg errorMessage)
    {
        var errorCode = errorMessage.val;
        return errorCode switch
        {
            MoveItErrorCodesMsg.COLLISION_CHECKING_UNAVAILABLE => "Collision checking unavailable",
            MoveItErrorCodesMsg.CONTROL_FAILED => "Control failed",
            MoveItErrorCodesMsg.FAILURE => "Failure",
            MoveItErrorCodesMsg.FRAME_TRANSFORM_FAILURE => "Frame transform failure",
            MoveItErrorCodesMsg.GOAL_CONSTRAINTS_VIOLATED => "Goal constraints violated",
            MoveItErrorCodesMsg.GOAL_IN_COLLISION => "Goal in collision",
            MoveItErrorCodesMsg.GOAL_VIOLATES_PATH_CONSTRAINTS => "Goal violates path constraints",
            MoveItErrorCodesMsg.INVALID_GOAL_CONSTRAINTS => "Invalid goal constraints",
            MoveItErrorCodesMsg.INVALID_GROUP_NAME => "Invalid group name",
            MoveItErrorCodesMsg.INVALID_LINK_NAME => "Invalid link name",
            MoveItErrorCodesMsg.INVALID_MOTION_PLAN => "Invalid motion plan",
            MoveItErrorCodesMsg.INVALID_OBJECT_NAME => "Invalid object name",
            MoveItErrorCodesMsg.INVALID_ROBOT_STATE => "Invalid robot state",
            MoveItErrorCodesMsg.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE => "Motion plan invalidated by environment change",
            MoveItErrorCodesMsg.NO_IK_SOLUTION => "No IK solution",
            MoveItErrorCodesMsg.PLANNING_FAILED => "Planning failed",
            MoveItErrorCodesMsg.PREEMPTED => "Preempted",
            MoveItErrorCodesMsg.ROBOT_STATE_STALE => "Robot state stale",
            MoveItErrorCodesMsg.SENSOR_INFO_STALE => "Sensor info stale",
            MoveItErrorCodesMsg.START_STATE_IN_COLLISION => "Start state in collision",
            MoveItErrorCodesMsg.START_STATE_VIOLATES_PATH_CONSTRAINTS => "Start state violates path constraints",
            MoveItErrorCodesMsg.SUCCESS => "Success",
            MoveItErrorCodesMsg.TIMED_OUT => "Timed out",
            MoveItErrorCodesMsg.UNABLE_TO_AQUIRE_SENSOR_DATA => "Unable to acquire sensor data",
            _ => "Invalid error message",
        };
    }
}

