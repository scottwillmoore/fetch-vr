using UnityEngine;

public class ShowChildren : MonoBehaviour
{
    [SerializeField] private GameObject childObject;

    void Start()
    {
        var gameRenderers = gameObject.GetComponentsInChildren<Renderer>();
        foreach (var renderer in gameRenderers)
        {
            renderer.enabled = false;
        }

        var childRenderers = childObject.GetComponentsInChildren<Renderer>();
        foreach (var renderer in childRenderers)
        {
            renderer.enabled = true;
        }
    }
}
