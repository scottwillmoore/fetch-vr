using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.UI;
using UnityEngine;

public class CompressedImageViewer : MonoBehaviour
{
    [SerializeField] private string compressedImageTopic;

    private RawImage rawImage;

    private ROSConnection rosConnection;

    public void Start()
    {
        rawImage = gameObject.GetComponent<RawImage>();
        Debug.Assert(rawImage != null);

        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<CompressedImageMsg>(compressedImageTopic, (compressedImage) =>
        {
            rawImage.texture = compressedImage.ToTexture2D();
        });
    }
}
