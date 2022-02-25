using UnityEngine;

public static class GameObjectExtensions
{
    public static void SetVisability(this GameObject gameObject, bool isVisable)
    {
        var renderers = gameObject.GetComponentsInChildren<Renderer>();
        foreach (var renderer in renderers)
        {
            renderer.enabled = isVisable;
        }
    }
}

