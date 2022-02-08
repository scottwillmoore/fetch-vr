using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class PickupAction : Action<PickupActionGoal, PickupActionResult, PickupActionFeedback, PickupGoal, PickupResult, PickupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupAction";
        public override string RosMessageName => k_RosMessageName;


        public PickupAction() : base()
        {
            this.action_goal = new PickupActionGoal();
            this.action_result = new PickupActionResult();
            this.action_feedback = new PickupActionFeedback();
        }

        public static PickupAction Deserialize(MessageDeserializer deserializer) => new PickupAction(deserializer);

        PickupAction(MessageDeserializer deserializer)
        {
            this.action_goal = PickupActionGoal.Deserialize(deserializer);
            this.action_result = PickupActionResult.Deserialize(deserializer);
            this.action_feedback = PickupActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
