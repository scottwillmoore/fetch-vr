using UnityEngine;

public class TransformSubscriber : MonoBehaviour
{
    [SerializeField] private string transformTopic = "/tf";

    [SerializeField] private string frameId = "base_link";

    private ArticulationBody rootArticulationBody;

    void Start()
    {
        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();
        foreach (var articulationBody in articulationBodies)
        {
            if (articulationBody.isRoot)
            {
                rootArticulationBody = articulationBody;
                break;
            }
        }
    }

    void Update()
    {
        var transformObject = TFSystem.GetOrCreateInstance().GetTransformObject(frameId, transformTopic);
        if (transformObject != null)
        {
            if (rootArticulationBody != null)
            {
                rootArticulationBody.TeleportRoot(transformObject.transform.position, transformObject.transform.rotation);
            }
            else
            {
                gameObject.transform.position = transformObject.transform.position;
                gameObject.transform.rotation = transformObject.transform.rotation;
            }
        }
    }
}
