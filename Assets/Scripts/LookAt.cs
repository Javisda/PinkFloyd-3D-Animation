using UnityEngine;
using UnityEngine.Animations.Rigging;

public class LookAt : MonoBehaviour
{

    private MultiAimConstraint mac;
    public Transform playerTransform;
    public Transform granProfesorTransform;
    // Start is called before the first frame update
    void Start()
    {
        mac = GetComponent<MultiAimConstraint>();
    }

    // Update is called once per frame
    void Update()
    {
        if (playerTransform.position.z < granProfesorTransform.position.z)
        {
            mac.weight = 0;
        }
        else {
            mac.weight = 1;
        }
    }
}
