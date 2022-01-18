using RosMessageTypes.Sensor;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class JointSubscriber : MonoBehaviour
{
    private ROSConnection rosConnection;

    private Dictionary<string, ArticulationBody> namedArticulationBodies;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<JointStateMsg>("robot/joint_states", JointStateSubscription);

        namedArticulationBodies = new Dictionary<string, ArticulationBody>();

        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();

        foreach (var articulationBody in articulationBodies)
        {
            if (articulationBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                var urdfJoint = articulationBody.gameObject.GetComponent<UrdfJointRevolute>();

                var linkName = urdfJoint.name;
                var jointName = urdfJoint.jointName;

                namedArticulationBodies[jointName] = articulationBody;
            }
        }
    }

    private void JointStateSubscription(JointStateMsg message)
    {
        for (var i = 0; i < message.name.Length; i++)
        {
            var name = message.name[i];
            var position = message.position[i];

            ArticulationBody articulationBody;
            if (namedArticulationBodies.TryGetValue(name, out articulationBody))
            {
                articulationBody.jointPosition = new ArticulationReducedSpace((float)position);
            }
        }
    }
}
