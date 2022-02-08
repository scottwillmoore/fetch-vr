using UnityEngine;

public class TransformAttachment : MonoBehaviour
{
    [SerializeField] private string transformTopic = "/tf";

    [SerializeField] private string frameId = "base_link";

    void Start()
    {
        var transformObject = TFSystem.GetOrCreateInstance().GetTransformObject(frameId, transformTopic);

        if (transformObject != null)
        {
            transform.parent = transformObject.transform;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
        }
    }
}
