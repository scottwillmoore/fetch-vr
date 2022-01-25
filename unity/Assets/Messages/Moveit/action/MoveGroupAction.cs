using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class MoveGroupAction : Action<MoveGroupActionGoal, MoveGroupActionResult, MoveGroupActionFeedback, MoveGroupGoal, MoveGroupResult, MoveGroupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupAction";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupAction() : base()
        {
            this.action_goal = new MoveGroupActionGoal();
            this.action_result = new MoveGroupActionResult();
            this.action_feedback = new MoveGroupActionFeedback();
        }

        public static MoveGroupAction Deserialize(MessageDeserializer deserializer) => new MoveGroupAction(deserializer);

        MoveGroupAction(MessageDeserializer deserializer)
        {
            this.action_goal = MoveGroupActionGoal.Deserialize(deserializer);
            this.action_result = MoveGroupActionResult.Deserialize(deserializer);
            this.action_feedback = MoveGroupActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
