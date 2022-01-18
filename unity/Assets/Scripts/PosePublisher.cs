using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class PosePublisher : MonoBehaviour
{
    public GameObject baxter;

    // public string buttonName;

    public string topicName;

    // public string serviceName;

    public float messageFrequency = 10.0f;

    private ROSConnection rosConnection;

    private TimeMsg rosTime;

    private float timeElapsed;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();

        rosTime = new TimeMsg();
        rosConnection.Subscribe<ClockMsg>("clock", ClockCallback);

        rosConnection.RegisterPublisher<PoseStampedMsg>(topicName);

        // rosConnection.RegisterRosService<MoveArmRequest, MoveArmResponse>(serviceName);
    }

    public void Update()
    {
        var message = new PoseStampedMsg();

        message.header.stamp = rosTime;
        message.header.frame_id = "base";

        message.pose.orientation = this.gameObject.transform.rotation.To<FLU>();

        var relativePosition = baxter.transform.InverseTransformPoint(this.gameObject.transform.position);
        message.pose.position = relativePosition.To<FLU>();

        timeElapsed += Time.deltaTime;

        var messagePeriod = 1.0f / messageFrequency;

        if (timeElapsed > messagePeriod)
        {
            rosConnection.Publish(topicName, message);

            timeElapsed = 0;
        }

        /*
        if (Input.GetButtonDown(buttonName))
        {
            var request = new MoveArmRequest();
            request.pose = message;
            rosConnection.SendServiceMessage<MoveArmResponse>(serviceName, request, MoveArmCallback);
        }
        */
    }

    private void ClockCallback(ClockMsg message)
    {
        rosTime = message.clock;
    }
}
