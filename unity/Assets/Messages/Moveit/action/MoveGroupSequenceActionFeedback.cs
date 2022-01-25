using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class MoveGroupSequenceActionFeedback : ActionFeedback<MoveGroupSequenceFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupSequenceActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupSequenceActionFeedback() : base()
        {
            this.feedback = new MoveGroupSequenceFeedback();
        }

        public MoveGroupSequenceActionFeedback(HeaderMsg header, GoalStatusMsg status, MoveGroupSequenceFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static MoveGroupSequenceActionFeedback Deserialize(MessageDeserializer deserializer) => new MoveGroupSequenceActionFeedback(deserializer);

        MoveGroupSequenceActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = MoveGroupSequenceFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
