using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.FetchAutoDock
{
    public class DockAction : Action<DockActionGoal, DockActionResult, DockActionFeedback, DockGoal, DockResult, DockFeedback>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/DockAction";
        public override string RosMessageName => k_RosMessageName;


        public DockAction() : base()
        {
            this.action_goal = new DockActionGoal();
            this.action_result = new DockActionResult();
            this.action_feedback = new DockActionFeedback();
        }

        public static DockAction Deserialize(MessageDeserializer deserializer) => new DockAction(deserializer);

        DockAction(MessageDeserializer deserializer)
        {
            this.action_goal = DockActionGoal.Deserialize(deserializer);
            this.action_result = DockActionResult.Deserialize(deserializer);
            this.action_feedback = DockActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
