using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.FetchAutoDock
{
    public class UndockAction : Action<UndockActionGoal, UndockActionResult, UndockActionFeedback, UndockGoal, UndockResult, UndockFeedback>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/UndockAction";
        public override string RosMessageName => k_RosMessageName;


        public UndockAction() : base()
        {
            this.action_goal = new UndockActionGoal();
            this.action_result = new UndockActionResult();
            this.action_feedback = new UndockActionFeedback();
        }

        public static UndockAction Deserialize(MessageDeserializer deserializer) => new UndockAction(deserializer);

        UndockAction(MessageDeserializer deserializer)
        {
            this.action_goal = UndockActionGoal.Deserialize(deserializer);
            this.action_result = UndockActionResult.Deserialize(deserializer);
            this.action_feedback = UndockActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
