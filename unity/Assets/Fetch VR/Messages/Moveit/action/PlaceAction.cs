using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class PlaceAction : Action<PlaceActionGoal, PlaceActionResult, PlaceActionFeedback, PlaceGoal, PlaceResult, PlaceFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceAction";
        public override string RosMessageName => k_RosMessageName;


        public PlaceAction() : base()
        {
            this.action_goal = new PlaceActionGoal();
            this.action_result = new PlaceActionResult();
            this.action_feedback = new PlaceActionFeedback();
        }

        public static PlaceAction Deserialize(MessageDeserializer deserializer) => new PlaceAction(deserializer);

        PlaceAction(MessageDeserializer deserializer)
        {
            this.action_goal = PlaceActionGoal.Deserialize(deserializer);
            this.action_result = PlaceActionResult.Deserialize(deserializer);
            this.action_feedback = PlaceActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
