using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class MoveGroupSequenceAction : Action<MoveGroupSequenceActionGoal, MoveGroupSequenceActionResult, MoveGroupSequenceActionFeedback, MoveGroupSequenceGoal, MoveGroupSequenceResult, MoveGroupSequenceFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupSequenceAction";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupSequenceAction() : base()
        {
            this.action_goal = new MoveGroupSequenceActionGoal();
            this.action_result = new MoveGroupSequenceActionResult();
            this.action_feedback = new MoveGroupSequenceActionFeedback();
        }

        public static MoveGroupSequenceAction Deserialize(MessageDeserializer deserializer) => new MoveGroupSequenceAction(deserializer);

        MoveGroupSequenceAction(MessageDeserializer deserializer)
        {
            this.action_goal = MoveGroupSequenceActionGoal.Deserialize(deserializer);
            this.action_result = MoveGroupSequenceActionResult.Deserialize(deserializer);
            this.action_feedback = MoveGroupSequenceActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
