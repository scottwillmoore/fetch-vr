using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class PickupActionResult : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionResult<PickupResult>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupActionResult";
        public override string RosMessageName => k_RosMessageName;


        public PickupActionResult() : base()
        {
            this.result = new PickupResult();
        }

        public PickupActionResult(HeaderMsg header, GoalStatusMsg status, PickupResult result) : base(header, status)
        {
            this.result = result;
        }
        public static PickupActionResult Deserialize(MessageDeserializer deserializer) => new PickupActionResult(deserializer);

        PickupActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = PickupResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
