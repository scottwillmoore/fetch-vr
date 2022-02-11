using UnityEngine;

public static class TransformExtensions
{
    public static Quaternion InverseTransformRotation(this Transform transform, Quaternion rotation)
    {
        return Quaternion.Inverse(transform.rotation) * rotation;
    }
}

