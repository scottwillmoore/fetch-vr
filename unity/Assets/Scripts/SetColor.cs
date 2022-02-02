using UnityEngine;

public class SetColor : MonoBehaviour
{
    [SerializeField]
    private Color newColor;

    void Start()
    {
        var renderers = gameObject.GetComponentsInChildren<Renderer>();
        foreach (var renderer in renderers)
        {
            renderer.material.color = newColor;
        }
    }
}
