using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class PickupActionFeedback : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionFeedback<PickupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public PickupActionFeedback() : base()
        {
            this.feedback = new PickupFeedback();
        }

        public PickupActionFeedback(HeaderMsg header, GoalStatusMsg status, PickupFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static PickupActionFeedback Deserialize(MessageDeserializer deserializer) => new PickupActionFeedback(deserializer);

        PickupActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = PickupFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
