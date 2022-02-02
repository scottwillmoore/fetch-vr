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
        MessageRegistry.Register(ExecuteTrajectoryActionGoal.k_RosMessageName, ExecuteTrajectoryActionGoal.Deserialize);
        MessageRegistry.Register(ExecuteTrajectoryActionResult.k_RosMessageName, ExecuteTrajectoryActionResult.Deserialize);
        MessageRegistry.Register(ExecuteTrajectoryActionFeedback.k_RosMessageName, ExecuteTrajectoryActionFeedback.Deserialize);

        MessageRegistry.Register(MoveGroupActionGoal.k_RosMessageName, ExecuteTrajectoryActionGoal.Deserialize);
        MessageRegistry.Register(MoveGroupActionResult.k_RosMessageName, ExecuteTrajectoryActionResult.Deserialize);
        MessageRegistry.Register(MoveGroupActionFeedback.k_RosMessageName, ExecuteTrajectoryActionFeedback.Deserialize);
    }

    public static double ToDouble(this DurationMsg durationMessage)
    {
        var duration = (double)durationMessage.sec;
        duration += secondsPerNanosecond * (double)durationMessage.nanosec;
        return duration;
    }
}

