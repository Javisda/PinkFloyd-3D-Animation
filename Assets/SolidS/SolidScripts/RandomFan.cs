using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomFan : MonoBehaviour
{
    public WindZone wind;
    [Range(0.0f, 20.0f)]
    [SerializeField] private float RandomizeXAxis;
    [Range(0.0f, 20.0f)]
    [SerializeField] private float RandomizeYAxis;

    void Update()
    {
        float xValue = Random.Range(-RandomizeXAxis, RandomizeXAxis);
        float yValue = Random.Range(-RandomizeYAxis, RandomizeYAxis);
        transform.Rotate(xValue * Time.deltaTime * wind.windMain, yValue * Time.deltaTime * wind.windMain, 4.0f * Time.deltaTime * wind.windMain);
    }
}
