using RosMessageTypes.Actionlib;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

class ROSActionResult<TResult> : Message
    where TResult : Message, new()
{
    public static string k_RosMessageName => typeof(TResult).GetField("k_RosMessageName").GetValue(null) + "ActionResult";

    public override string RosMessageName => k_RosMessageName;

    public HeaderMsg header;
    public GoalStatusMsg status;
    public TResult result;

    public ROSActionResult() : base()
    {
        this.header = new HeaderMsg();
        this.status = new GoalStatusMsg();
        this.result = new TResult();
    }

    public ROSActionResult(HeaderMsg header, GoalStatusMsg status, TResult result) : base()
    {
        this.header = header;
        this.status = status;
        this.result = result;
    }

    public static ROSActionResult<TResult> Deserialize(MessageDeserializer deserializer) => new ROSActionResult<TResult>(deserializer);

    public ROSActionResult(MessageDeserializer deserializer) : base()
    {
        this.header = HeaderMsg.Deserialize(deserializer);
        this.status = GoalStatusMsg.Deserialize(deserializer);
        this.result = (TResult)typeof(TResult).GetMethod("Deserialize", new[] { typeof(MessageDeserializer) }).Invoke(null, new[] { deserializer });
    }

    public override void SerializeTo(MessageSerializer serializer)
    {
        serializer.Write(this.header);
        serializer.Write(this.status);
        serializer.Write(this.result);
    }
}
