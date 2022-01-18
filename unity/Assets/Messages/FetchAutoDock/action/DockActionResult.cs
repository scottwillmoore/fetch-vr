using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class DockActionResult : ActionResult<DockResult>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/DockActionResult";
        public override string RosMessageName => k_RosMessageName;


        public DockActionResult() : base()
        {
            this.result = new DockResult();
        }

        public DockActionResult(HeaderMsg header, GoalStatusMsg status, DockResult result) : base(header, status)
        {
            this.result = result;
        }
        public static DockActionResult Deserialize(MessageDeserializer deserializer) => new DockActionResult(deserializer);

        DockActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = DockResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
