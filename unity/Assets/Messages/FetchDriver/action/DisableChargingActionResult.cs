using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchDriver
{
    public class DisableChargingActionResult : ActionResult<DisableChargingResult>
    {
        public const string k_RosMessageName = "fetch_driver_msgs/DisableChargingActionResult";
        public override string RosMessageName => k_RosMessageName;


        public DisableChargingActionResult() : base()
        {
            this.result = new DisableChargingResult();
        }

        public DisableChargingActionResult(HeaderMsg header, GoalStatusMsg status, DisableChargingResult result) : base(header, status)
        {
            this.result = result;
        }
        public static DisableChargingActionResult Deserialize(MessageDeserializer deserializer) => new DisableChargingActionResult(deserializer);

        DisableChargingActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = DisableChargingResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
