using RosMessageTypes.Aruco;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ArucoPositionSubscriber : MonoBehaviour
{
    [SerializeField] private string arucoTopic = "/aruco_marker_publisher/markers";

    private ROSConnection rosConnection;

    public GameObject tablePrefab;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<MarkerArrayMsg>(arucoTopic, ArucoCallback);
    }

    private void ArucoCallback(MarkerArrayMsg message)
    {
        // Debug.Log(message);

        // table size: w80 l180 h77

        // get transforms of prefab
        var zeroPosition = new Vector3(0, 0, 0);    // change to just in front of robot
        var zeroOrientation = Quaternion.Euler(-90, 0, 0);
        var modelSize = gameObject.GetComponentInChildren<MeshFilter>().mesh.bounds.size;
        // var prefabRotation = tablePrefab.transform.rotation.eulerAngles;

        gameObject.transform.localPosition = zeroPosition;
        gameObject.transform.localRotation = zeroOrientation;
        gameObject.transform.localScale = new Vector3 (80/modelSize[0], 180/modelSize[1], 77/modelSize[2]);
        gameObject.transform.Translate(0, 0, -0.8f);

        // get coordinates from message for aruco marker
        var position = message.markers[0].pose.pose.position.From<FLU>();
        var orientation = message.markers[0].pose.pose.orientation.From<FLU>();
        // Debug.Log(orientation.eulerAngles);

        gameObject.transform.parent.gameObject.transform.localPosition = position;
        gameObject.transform.parent.gameObject.transform.localRotation = orientation;

/*
        // get transforms of prefab
        var prefabPosition = tablePrefab.transform.position;
        var prefabOrientation = tablePrefab.transform.rotation;
        // var prefabRotation = tablePrefab.transform.rotation.eulerAngles;

        // spawn table
        var tableObj = Instantiate(tablePrefab, prefabPosition, prefabOrientation);

        tableObj.gameObject.transform.SetParent(gameObject.transform, false);
        tableObj.gameObject.transform.position = position;
        tableObj.gameObject.transform.rotation = orientation;
        // tableObj.gameObject.transform.Translate(position);
        // tableObj.gameObject.transform.Rotate(orientation.eulerAngles);
        tableObj.gameObject.transform.SetParent(gameObject.gameObject.transform, false);
*/


        rosConnection.Unsubscribe(arucoTopic);
    }
}
