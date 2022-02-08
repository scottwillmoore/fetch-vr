using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class MoveGroupActionFeedback : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionFeedback<MoveGroupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionFeedback() : base()
        {
            this.feedback = new MoveGroupFeedback();
        }

        public MoveGroupActionFeedback(HeaderMsg header, GoalStatusMsg status, MoveGroupFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static MoveGroupActionFeedback Deserialize(MessageDeserializer deserializer) => new MoveGroupActionFeedback(deserializer);

        MoveGroupActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = MoveGroupFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
