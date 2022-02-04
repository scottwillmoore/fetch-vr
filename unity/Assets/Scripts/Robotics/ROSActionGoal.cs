using RosMessageTypes.Actionlib;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

class ROSActionGoal<TGoal> : Message
    where TGoal : Message, new()
{
    public static string k_RosMessageName => typeof(TGoal).GetField("k_RosMessageName").GetValue(null) + "ActionGoal";

    public override string RosMessageName => k_RosMessageName;

    public HeaderMsg header;
    public GoalIDMsg goal_id;
    public TGoal goal;

    public ROSActionGoal() : base()
    {
        this.header = new HeaderMsg();
        this.goal_id = new GoalIDMsg();
        this.goal = new TGoal();
    }

    public ROSActionGoal(HeaderMsg header, GoalIDMsg goal_id, TGoal goal) : base()
    {
        this.header = header;
        this.goal_id = goal_id;
        this.goal = goal;
    }

    public static ROSActionGoal<TGoal> Deserialize(MessageDeserializer deserializer) => new ROSActionGoal<TGoal>(deserializer);

    public ROSActionGoal(MessageDeserializer deserializer) : base()
    {
        this.header = HeaderMsg.Deserialize(deserializer);
        this.goal_id = GoalIDMsg.Deserialize(deserializer);
        this.goal = (TGoal)typeof(TGoal).GetMethod("Deserialize", new[] { typeof(MessageDeserializer) }).Invoke(null, new[] { deserializer });
    }

    public override void SerializeTo(MessageSerializer serializer)
    {
        serializer.Write(this.header);
        serializer.Write(this.goal_id);
        serializer.Write(this.goal);
    }
}
