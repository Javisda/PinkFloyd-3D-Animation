using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HammerAudioController : MonoBehaviour
{

    public AudioSource audioSource;
    public Transform playerTransform;

    // Start is called before the first frame update
    void Start()
    {
        audioSource = GetComponent<AudioSource>();
    }

    // Update is called once per frame
    void Update()
    {
        float maxDistance = 250.0f;
        float distance = Vector3.Distance(transform.position, playerTransform.position);

        if (distance > maxDistance * 2)
        {
            audioSource.volume = 0.0f;
        }
        else { 
            float volume = maxDistance / distance / 100;
            audioSource.volume = volume;
        }

    }
}
