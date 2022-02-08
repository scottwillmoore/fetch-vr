using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Moveit
{
    public class MoveGroupActionGoal : Unity.Robotics.ROSTCPConnector.MessageGeneration.ActionGoal<MoveGroupGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionGoal() : base()
        {
            this.goal = new MoveGroupGoal();
        }

        public MoveGroupActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveGroupGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static MoveGroupActionGoal Deserialize(MessageDeserializer deserializer) => new MoveGroupActionGoal(deserializer);

        MoveGroupActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveGroupGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
