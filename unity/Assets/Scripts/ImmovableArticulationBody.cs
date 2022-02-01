using UnityEngine;

public class ImmovableArticulationBody : MonoBehaviour
{
    public void Start()
    {
        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();
        foreach (var articulationBody in articulationBodies)
        {
            if (articulationBody.isRoot)
            {
                articulationBody.immovable = true;
            }
            else
            {
                articulationBody.jointFriction = float.PositiveInfinity;
            }
        }
    }
}
