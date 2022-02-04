using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class MoveGroupActionResult : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionResult<MoveGroupResult>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionResult() : base()
        {
            this.result = new MoveGroupResult();
        }

        public MoveGroupActionResult(HeaderMsg header, GoalStatusMsg status, MoveGroupResult result) : base(header, status)
        {
            this.result = result;
        }
        public static MoveGroupActionResult Deserialize(MessageDeserializer deserializer) => new MoveGroupActionResult(deserializer);

        MoveGroupActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = MoveGroupResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
