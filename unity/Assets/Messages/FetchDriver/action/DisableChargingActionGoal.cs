using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchDriver
{
    public class DisableChargingActionGoal : ActionGoal<DisableChargingGoal>
    {
        public const string k_RosMessageName = "fetch_driver_msgs/DisableChargingActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public DisableChargingActionGoal() : base()
        {
            this.goal = new DisableChargingGoal();
        }

        public DisableChargingActionGoal(HeaderMsg header, GoalIDMsg goal_id, DisableChargingGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static DisableChargingActionGoal Deserialize(MessageDeserializer deserializer) => new DisableChargingActionGoal(deserializer);

        DisableChargingActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = DisableChargingGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
