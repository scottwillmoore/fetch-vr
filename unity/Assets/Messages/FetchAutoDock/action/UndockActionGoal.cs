using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class UndockActionGoal : ActionGoal<UndockGoal>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/UndockActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public UndockActionGoal() : base()
        {
            this.goal = new UndockGoal();
        }

        public UndockActionGoal(HeaderMsg header, GoalIDMsg goal_id, UndockGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static UndockActionGoal Deserialize(MessageDeserializer deserializer) => new UndockActionGoal(deserializer);

        UndockActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = UndockGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
