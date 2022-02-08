using RosMessageTypes.Actionlib;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

class ROSActionFeedback<TFeedback> : Message
    where TFeedback : Message, new()
{
    public static string k_RosMessageName => typeof(TFeedback).GetField("k_RosMessageName").GetValue(null) + "ActionFeedback";

    public override string RosMessageName => k_RosMessageName;

    public HeaderMsg header;
    public GoalStatusMsg status;
    public TFeedback feedback;

    public ROSActionFeedback() : base()
    {
        this.header = new HeaderMsg();
        this.status = new GoalStatusMsg();
        this.feedback = new TFeedback();
    }

    public ROSActionFeedback(HeaderMsg header, GoalStatusMsg status, TFeedback feedback) : base()
    {
        this.header = header;
        this.status = status;
        this.feedback = feedback;
    }

    public static ROSActionFeedback<TFeedback> Deserialize(MessageDeserializer deserializer) => new ROSActionFeedback<TFeedback>(deserializer);

    public ROSActionFeedback(MessageDeserializer deserializer) : base()
    {
        this.header = HeaderMsg.Deserialize(deserializer);
        this.status = GoalStatusMsg.Deserialize(deserializer);
        this.feedback = (TFeedback)typeof(TFeedback).GetMethod("Deserialize", new[] { typeof(MessageDeserializer) }).Invoke(null, new[] { deserializer });
    }

    public override void SerializeTo(MessageSerializer serializer)
    {
        serializer.Write(this.header);
        serializer.Write(this.status);
        serializer.Write(this.feedback);
    }
}

