using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class ExecuteTrajectoryAction : Action<ExecuteTrajectoryActionGoal, ExecuteTrajectoryActionResult, ExecuteTrajectoryActionFeedback, ExecuteTrajectoryGoal, ExecuteTrajectoryResult, ExecuteTrajectoryFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/ExecuteTrajectoryAction";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTrajectoryAction() : base()
        {
            this.action_goal = new ExecuteTrajectoryActionGoal();
            this.action_result = new ExecuteTrajectoryActionResult();
            this.action_feedback = new ExecuteTrajectoryActionFeedback();
        }

        public static ExecuteTrajectoryAction Deserialize(MessageDeserializer deserializer) => new ExecuteTrajectoryAction(deserializer);

        ExecuteTrajectoryAction(MessageDeserializer deserializer)
        {
            this.action_goal = ExecuteTrajectoryActionGoal.Deserialize(deserializer);
            this.action_result = ExecuteTrajectoryActionResult.Deserialize(deserializer);
            this.action_feedback = ExecuteTrajectoryActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
