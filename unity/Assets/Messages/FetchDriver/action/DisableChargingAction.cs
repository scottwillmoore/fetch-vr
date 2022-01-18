using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.FetchDriver
{
    public class DisableChargingAction : Action<DisableChargingActionGoal, DisableChargingActionResult, DisableChargingActionFeedback, DisableChargingGoal, DisableChargingResult, DisableChargingFeedback>
    {
        public const string k_RosMessageName = "fetch_driver_msgs/DisableChargingAction";
        public override string RosMessageName => k_RosMessageName;


        public DisableChargingAction() : base()
        {
            this.action_goal = new DisableChargingActionGoal();
            this.action_result = new DisableChargingActionResult();
            this.action_feedback = new DisableChargingActionFeedback();
        }

        public static DisableChargingAction Deserialize(MessageDeserializer deserializer) => new DisableChargingAction(deserializer);

        DisableChargingAction(MessageDeserializer deserializer)
        {
            this.action_goal = DisableChargingActionGoal.Deserialize(deserializer);
            this.action_result = DisableChargingActionResult.Deserialize(deserializer);
            this.action_feedback = DisableChargingActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
