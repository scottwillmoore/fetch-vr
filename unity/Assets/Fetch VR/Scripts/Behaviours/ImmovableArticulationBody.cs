using UnityEngine;

public class ImmovableArticulationBody : MonoBehaviour
{
    private ArticulationBody rootArticulationBody;

    public void Start()
    {
        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();
        foreach (var articulationBody in articulationBodies)
        {
            if (articulationBody.isRoot)
            {
                articulationBody.immovable = true;
                rootArticulationBody = articulationBody;
            }
            else
            {
                articulationBody.jointFriction = float.PositiveInfinity;
            }
        }
    }

    public void Update()
    {
        rootArticulationBody.TeleportRoot(gameObject.transform.parent.position, gameObject.transform.parent.rotation);
    }
}
