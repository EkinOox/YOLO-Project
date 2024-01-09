using UnityEngine;

public class ResizeBall : MonoBehaviour
{
    public UdpSocket udpSocket;
    public GameObject Sphere;
    public float minBallSize = 1.0f;
    public float maxBallSize = 20.0f;

    void Start()
    {
        // Assurez-vous d'ajouter un composant UdpSocket à l'objet qui a ce script ResizeBall
        udpSocket = GetComponent<UdpSocket>();
    }

    void Update()
    {
        // Accéder à wristDistance via l'instance de UdpSocket
        float wristDistance = udpSocket.wristDistance;

        // Ajuster la taille de la balle en fonction de la distance entre les poignets
        if (wristDistance > 0)
        {
            Sphere.transform.localScale = new Vector3(1, 1, 1) * wristDistance / 100;
        }
    }
}
