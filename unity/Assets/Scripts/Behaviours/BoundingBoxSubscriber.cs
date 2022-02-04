using RosMessageTypes.JskRecognition;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class BoundingBoxSubscriber : MonoBehaviour
{
    [SerializeField] private string boxTopic = "/segmentation/bounding_boxes";

    private ROSConnection rosConnection;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<BoundingBoxArrayMsg>(boxTopic, BoundingBoxesCallback);
    }

    private void BoundingBoxesCallback(BoundingBoxArrayMsg message)
    {
        // Debug.Log(message);
        // destroy previous exisiting bounding boxes
        GameObject[] oldBoxes = GameObject.FindGameObjectsWithTag("Bounding Box");
        foreach (GameObject oldBox in oldBoxes)
        {
            GameObject.Destroy(oldBox);
        }

        // get coordinates from message for each box
        foreach (var box in message.boxes)
        {
            // Debug.Log(box);

            var position = box.pose.position.From<FLU>();
            var orientation = box.pose.orientation.From<FLU>(); // quaternion
            var dimensions = box.dimensions.From<FLU>();

            // draw to unity with gizmos

            // create cube object in unity
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.tag = "Bounding Box";
            cube.transform.position = position;
            cube.transform.rotation = orientation;
            cube.transform.localScale = dimensions;
            cube.transform.SetParent(gameObject.transform, false);

            // make cube transparent
            Material mat = cube.GetComponent<Renderer>().material;
            float alpha = 0.5f;
            Color transparentColour = new Color(0.6f, 0.6f, 0.6f, alpha);//mat.color.r, mat.color.g, mat.color.b, alpha);
            mat.SetColor("_Color", transparentColour);
            mat.shader = Shader.Find("Transparent/Diffuse");
        }
    }

    // public void Update()
    // {
    //     GameObject[] boxes = GameObject.FindGameObjectsWithTag("Bounding Box");
    //     foreach(GameObject box in boxes)
    //     {
    //         Material mat = box.GetComponent<Renderer>().material;
    //         float alpha = 0.5f;
    //         Color transparentColour = new Color(1,1,1,alpha);//mat.color.r, mat.color.g, mat.color.b, alpha);
    //         mat.SetColor("_Color", transparentColour);
    //     }
    // }

}
