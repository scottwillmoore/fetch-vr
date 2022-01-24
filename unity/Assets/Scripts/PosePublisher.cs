using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

using static OVRInput;

public class PosePublisher : MonoBehaviour
{
    [Header("Coordinate Frame")]
    [SerializeField] private string frameId;
    [SerializeField] private GameObject frameObject;

    [Header("Clock Topic")]
    [SerializeField]
    private string clockTopicName;

    [Header("Pose Topic")]
    [SerializeField] private string poseTopicName;
    [SerializeField] private float poseTopicFrequency = 10.0f;

    [Header("Motion Plan Service")]
    [SerializeField] private string motionPlanServiceName;
    [SerializeField] private RawButton motionPlanServiceButton;

    [Header("Execute Trajectory Action")]
    [SerializeField] private string executeTrajectoryActionName;

    private ROSConnection rosConnection;

    private TimeMsg rosTime;

    private float timeElapsed;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();

        rosTime = new TimeMsg();
        rosConnection.Subscribe<ClockMsg>(clockTopicName, ClockCallback);

        rosConnection.RegisterPublisher<PoseStampedMsg>(poseTopicName);

        // rosConnection.RegisterRosService<MoveArmRequest, MoveArmResponse>(serviceName);
    }

    public void Update()
    {
        var relativePosition = frameObject.transform.InverseTransformPoint(gameObject.transform.position);
        // var relativePosition = gameObject.transform.position - frameObject.transform.position;
        var relativeRotation = Quaternion.Inverse(gameObject.transform.rotation) * frameObject.transform.rotation;

        var message = new PoseStampedMsg();
        message.header.frame_id = frameId;
        message.header.stamp = rosTime;
        message.pose.orientation = relativeRotation.To<FLU>();
        message.pose.position = relativePosition.To<FLU>();

        timeElapsed += Time.deltaTime;

        var messagePeriod = 1.0f / poseTopicFrequency;

        if (timeElapsed > messagePeriod)
        {
            rosConnection.Publish(poseTopicName, message);

            timeElapsed = 0;
        }

        if (OVRInput.GetDown(motionPlanServiceButton))
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
