using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class DockActionFeedback : ActionFeedback<DockFeedback>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/DockActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public DockActionFeedback() : base()
        {
            this.feedback = new DockFeedback();
        }

        public DockActionFeedback(HeaderMsg header, GoalStatusMsg status, DockFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static DockActionFeedback Deserialize(MessageDeserializer deserializer) => new DockActionFeedback(deserializer);

        DockActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = DockFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
