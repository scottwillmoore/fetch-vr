using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class UndockActionFeedback : ActionFeedback<UndockFeedback>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/UndockActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public UndockActionFeedback() : base()
        {
            this.feedback = new UndockFeedback();
        }

        public UndockActionFeedback(HeaderMsg header, GoalStatusMsg status, UndockFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static UndockActionFeedback Deserialize(MessageDeserializer deserializer) => new UndockActionFeedback(deserializer);

        UndockActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = UndockFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
