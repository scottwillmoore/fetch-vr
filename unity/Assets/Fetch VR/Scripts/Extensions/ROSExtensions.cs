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
}

