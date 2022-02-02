using System.Collections.Generic;
using UnityEngine;

public class Tint : MonoBehaviour
{
    [SerializeField]
    private Color tintColor;

    [SerializeField]
    private float tintStrength;

    private List<(Renderer renderer, Color oldColor)> initialRenderers;

    public void OnValidate()
    {
        SetTint();
    }

    public void Start()
    {
        initialRenderers = new List<(Renderer renderer, Color color)>();

        var renderers = gameObject.GetComponentsInChildren<Renderer>();
        foreach (var renderer in renderers)
        {
            var oldColor = renderer.material.color;
            initialRenderers.Add((renderer, oldColor));
        }

        SetTint();
    }

    private void SetTint()
    {
        if (initialRenderers == null)
        {
            return;
        }

        foreach (var (renderer, oldColor) in initialRenderers)
        {
            var newColor = Color.Lerp(oldColor, tintColor, tintStrength);
            newColor.a = tintColor.a;

            renderer.material.color = newColor;
            if (newColor.a < 1.0f)
            {
                renderer.material.SetTransparent();
            }
            else
            {
                renderer.material.SetOpaque();
            }
        }
    }
}
