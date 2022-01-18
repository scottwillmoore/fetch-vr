using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class UndockActionResult : ActionResult<UndockResult>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/UndockActionResult";
        public override string RosMessageName => k_RosMessageName;


        public UndockActionResult() : base()
        {
            this.result = new UndockResult();
        }

        public UndockActionResult(HeaderMsg header, GoalStatusMsg status, UndockResult result) : base(header, status)
        {
            this.result = result;
        }
        public static UndockActionResult Deserialize(MessageDeserializer deserializer) => new UndockActionResult(deserializer);

        UndockActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = UndockResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
