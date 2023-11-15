using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]

public class moveBall : MonoBehaviour
{
    public float xForce = 10.0f;
    public float zForce = 10.0f;
    public float yForce = 500.0f;
    public float minBallSize = 1.0f;
    public float maxBallSize = 20.0f;

    // Référence à l'instance de UdpSocket
    public UdpSocket udpSocket;
    public GameObject Sphere;
    void Start()
    {
        // Assurez-vous d'ajouter un composant UdpSocket à l'objet qui a ce script moveBall
        udpSocket = GetComponent<UdpSocket>();
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

        // Accéder à wristDistance via l'instance de UdpSocket
        float wristDistance = udpSocket.wristDistance;

        // Ajuster la taille de la balle en fonction de la distance entre les poignets
        if (wristDistance > 0)
        {
            Sphere.transform.localScale = new Vector3(1, 1, 1) * wristDistance / 100;
        }
    }
}
