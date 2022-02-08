using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class PlaceActionResult : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionResult<PlaceResult>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceActionResult";
        public override string RosMessageName => k_RosMessageName;


        public PlaceActionResult() : base()
        {
            this.result = new PlaceResult();
        }

        public PlaceActionResult(HeaderMsg header, GoalStatusMsg status, PlaceResult result) : base(header, status)
        {
            this.result = result;
        }
        public static PlaceActionResult Deserialize(MessageDeserializer deserializer) => new PlaceActionResult(deserializer);

        PlaceActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = PlaceResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
