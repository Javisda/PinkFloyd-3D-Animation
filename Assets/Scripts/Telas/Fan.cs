using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fan : MonoBehaviour
{
    public WindZone wind;

    void Update()
    {
        transform.Rotate(0.0f, 0.0f, 4.0f * Time.deltaTime * wind.windMain);
    }
}
