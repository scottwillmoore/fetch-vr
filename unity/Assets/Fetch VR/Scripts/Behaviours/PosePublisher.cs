using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class PosePublisher : MonoBehaviour
{
    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private TFSystem tfSystem;

    [SerializeField] private string frameId;

    [SerializeField] private string posePublisherName;

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        tfSystem = TFSystem.GetOrCreateInstance();

        rosConnection.RegisterPublisher<PoseStampedMsg>(posePublisherName);
    }

    void Update()
    {
        var currentPose = GetCurrentPose();
        rosConnection.Publish(posePublisherName, currentPose);

    }

    private PoseStampedMsg GetCurrentPose()
    {
        var referenceObject = tfSystem.GetTransformObject(frameId);

        var relativePosition = referenceObject.transform.InverseTransformPoint(gameObject.transform.position);
        var relativeRotation = referenceObject.transform.InverseTransformRotation(gameObject.transform.rotation);

        var currentPose = new PoseStampedMsg();
        currentPose.header.stamp = rosTime.Now();
        currentPose.header.frame_id = frameId;

        currentPose.pose.position = relativePosition.To<FLU>();
        currentPose.pose.orientation = relativeRotation.To<FLU>();

        return currentPose;
    }
}
