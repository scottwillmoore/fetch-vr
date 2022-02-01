using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class PlaceActionFeedback : ActionFeedback<PlaceFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public PlaceActionFeedback() : base()
        {
            this.feedback = new PlaceFeedback();
        }

        public PlaceActionFeedback(HeaderMsg header, GoalStatusMsg status, PlaceFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static PlaceActionFeedback Deserialize(MessageDeserializer deserializer) => new PlaceActionFeedback(deserializer);

        PlaceActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = PlaceFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
