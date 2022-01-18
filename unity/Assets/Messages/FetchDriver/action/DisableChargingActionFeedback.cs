using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchDriver
{
    public class DisableChargingActionFeedback : ActionFeedback<DisableChargingFeedback>
    {
        public const string k_RosMessageName = "fetch_driver_msgs/DisableChargingActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public DisableChargingActionFeedback() : base()
        {
            this.feedback = new DisableChargingFeedback();
        }

        public DisableChargingActionFeedback(HeaderMsg header, GoalStatusMsg status, DisableChargingFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static DisableChargingActionFeedback Deserialize(MessageDeserializer deserializer) => new DisableChargingActionFeedback(deserializer);

        DisableChargingActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = DisableChargingFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
