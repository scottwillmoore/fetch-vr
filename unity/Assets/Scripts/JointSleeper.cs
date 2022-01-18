using UnityEngine;

public class JointSleeper : MonoBehaviour
{
    public void FixedUpdate()
    {
        var articulationBodies = this.GetComponentsInChildren<ArticulationBody>();

        foreach (var articulationBody in articulationBodies)
        {
            articulationBody.Sleep();
        }
    }
}
