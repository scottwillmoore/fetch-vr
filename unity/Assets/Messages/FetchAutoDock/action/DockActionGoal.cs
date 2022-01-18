using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.FetchAutoDock
{
    public class DockActionGoal : ActionGoal<DockGoal>
    {
        public const string k_RosMessageName = "fetch_auto_dock_msgs/DockActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public DockActionGoal() : base()
        {
            this.goal = new DockGoal();
        }

        public DockActionGoal(HeaderMsg header, GoalIDMsg goal_id, DockGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static DockActionGoal Deserialize(MessageDeserializer deserializer) => new DockActionGoal(deserializer);

        DockActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = DockGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
