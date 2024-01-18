using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]

public class moveBall : MonoBehaviour
{
    public float xForce = 10.0f;
    public float zForce = 10.0f;
    public float yForce = 500.0f;
    public GameObject Sphere;
    void Start()
    {
    }

    void Update()
    {
        float x = 0.0f;
        if (Input.GetKey(KeyCode.Q))
        {
            x = x - xForce;
        }

        if (Input.GetKey(KeyCode.D))
        {
            x = x + xForce;
        }

        float z = 0.0f;
        if (Input.GetKey(KeyCode.S))
        {
            z = z - zForce;
        }

        if (Input.GetKey(KeyCode.Z))
        {
            z = z + zForce;
        }

        float y = 0.0f;
        if (Input.GetKeyDown(KeyCode.Space))
        {
            y = yForce;
        }

        GetComponent<Rigidbody>().AddForce(x, y, z);
    }
}
