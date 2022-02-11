using UnityEngine;

public class TransformSubscriber : MonoBehaviour
{
    [SerializeField] private string frameId;

    private GameObject transformObject;

    void Update()
    {
        if (transformObject == null)
        {
            transformObject = TFSystem.GetOrCreateInstance().GetTransformObject(frameId);
        }
        else
        {
            gameObject.transform.position = transformObject.transform.position;
            gameObject.transform.rotation = transformObject.transform.rotation;
        }
    }
}
