using RosMessageTypes.Moveit;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class JointTrajectoryAnimator : MonoBehaviour
{
    [SerializeField]
    private string displayPlannedPathTopic = "/move_group/display_planned_path";

    private ROSConnection rosConnection;

    private Dictionary<string, ArticulationBody> namedArticulationBodies;

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

    private void DisplayPlannedPathCallback(DisplayTrajectoryMsg displayTrajectory)
    {
        var lastRobotTrajectory = displayTrajectory.trajectory[displayTrajectory.trajectory.Length - 1];

        var jointTrajectory = lastRobotTrajectory.joint_trajectory;
        var lastJointTrajectoryPoint = jointTrajectory.points[jointTrajectory.points.Length - 1];

        for (var i = 0; i < jointTrajectory.joint_names.Length; i++)
        {
            var name = jointTrajectory.joint_names[i];
            var position = lastJointTrajectoryPoint.positions[i];

            if (namedArticulationBodies.TryGetValue(name, out ArticulationBody articulationBody))
            {
                articulationBody.jointPosition = new ArticulationReducedSpace((float)position);
            }
        }
    }
}
