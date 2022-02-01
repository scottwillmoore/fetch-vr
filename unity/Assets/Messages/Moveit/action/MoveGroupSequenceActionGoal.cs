using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class MoveGroupSequenceActionGoal : ActionGoal<MoveGroupSequenceGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupSequenceActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupSequenceActionGoal() : base()
        {
            this.goal = new MoveGroupSequenceGoal();
        }

        public MoveGroupSequenceActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveGroupSequenceGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static MoveGroupSequenceActionGoal Deserialize(MessageDeserializer deserializer) => new MoveGroupSequenceActionGoal(deserializer);

        MoveGroupSequenceActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveGroupSequenceGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
