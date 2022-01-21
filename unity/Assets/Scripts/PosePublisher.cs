using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

using static OVRInput;

public class PosePublisher : MonoBehaviour
{
    [Header("Reference Frame")]
    [SerializeField] private string frameId;
    [SerializeField] private GameObject frameObject;

    [Header("Published Topic")]
    [SerializeField] private string topicName;
    [SerializeField] private float topicFrequency = 10.0f;

    [Header("Motion Plan Service")]
    [SerializeField] private string serviceName;
    [SerializeField] private RawButton serviceButton;

    [Header("Execute Trajectory Action")]
    [SerializeField] private string actionName;

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
        var relativePosition = frameObject.transform.InverseTransformPoint(this.gameObject.transform.position);

        var message = new PoseStampedMsg();
        message.header.frame_id = frameId;
        message.header.stamp = rosTime;
        message.pose.orientation = this.gameObject.transform.rotation.To<FLU>();
        message.pose.position = relativePosition.To<FLU>();

        timeElapsed += Time.deltaTime;

        var messagePeriod = 1.0f / topicFrequency;

        if (timeElapsed > messagePeriod)
        {
            rosConnection.Publish(topicName, message);

            timeElapsed = 0;
        }

        if (OVRInput.GetDown(serviceButton))
        {
            Debug.Log("!!!");

            // var request = new MoveArmRequest();
            // request.pose = message;
            // rosConnection.SendServiceMessage<MoveArmResponse>(serviceName, request, MoveArmCallback);
        }
    }

    private void ClockCallback(ClockMsg message)
    {
        rosTime = message.clock;
    }
}
