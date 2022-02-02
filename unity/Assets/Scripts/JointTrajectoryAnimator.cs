using RosMessageTypes.Moveit;
using RosMessageTypes.Trajectory;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class JointTrajectoryAnimator : MonoBehaviour
{
    [SerializeField]
    private string displayPlannedPathTopic = "/move_group/display_planned_path";

    private ROSConnection rosConnection;

    private Dictionary<string, ArticulationBody> namedArticulationBodies;

    private JointTrajectoryMsg jointTrajectory;

    private double animationDuration;
    private double animationStartTime;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<DisplayTrajectoryMsg>(displayPlannedPathTopic, DisplayPlannedPathCallback);

        namedArticulationBodies = new Dictionary<string, ArticulationBody>();

        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();
        foreach (var articulationBody in articulationBodies)
        {
            if (articulationBody.gameObject.TryGetComponent<UrdfJoint>(out UrdfJoint urdfJoint))
            {
                namedArticulationBodies[urdfJoint.jointName] = articulationBody;
            }
        }
    }

    public void Update()
    {
        if (jointTrajectory == null)
        {
            return;
        }

        var currentTime = Time.timeAsDouble;
        var elapsedTime = currentTime - animationStartTime;

        var animationElapsedTime = elapsedTime % animationDuration;

        JointTrajectoryPointMsg currentPoint = jointTrajectory.points[0];

        var reversedPoints = Enumerable.Reverse(jointTrajectory.points);
        foreach (var point in reversedPoints)
        {
            var pointElapsedTime = point.time_from_start.ToDouble();
            if (pointElapsedTime < animationElapsedTime)
            {
                currentPoint = point;
                break;
            }
        }

        for (var i = 0; i < jointTrajectory.joint_names.Length; i++)
        {
            var name = jointTrajectory.joint_names[i];
            var position = currentPoint.positions[i];
            if (namedArticulationBodies.TryGetValue(name, out ArticulationBody articulationBody))
            {
                articulationBody.jointPosition = new ArticulationReducedSpace((float)position);
            }
        }
    }

    private void DisplayPlannedPathCallback(DisplayTrajectoryMsg displayTrajectory)
    {
        Debug.Assert(displayTrajectory.trajectory.Length > 0);
        var robotTrajectory = displayTrajectory.trajectory[0];

        jointTrajectory = robotTrajectory.joint_trajectory;

        Debug.Assert(displayTrajectory.trajectory.Length > 0);
        var lastJointTrajectoryPoint = jointTrajectory.points[jointTrajectory.points.Length - 1];

        animationDuration = lastJointTrajectoryPoint.time_from_start.ToDouble();
        Debug.Assert(animationDuration > 0.0);

        animationStartTime = Time.timeAsDouble;
    }
}
