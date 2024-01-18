using UnityEngine;

public class ChangeColor : MonoBehaviour
{
    public UdpSocket udpSocket;
    public GameObject Sphere;
    public float decibelThreshold = 70.0f;
    public Color highDecibelColor = Color.red;
    public Color defaultColor = Color.white;

    private Color currentColor;  // Stocke la couleur actuelle pour v�rifier les changements

    void Start()
    {
        // Assurez-vous d'ajouter un composant UdpSocket � l'objet qui a ce script ChangeColor
        udpSocket = GetComponent<UdpSocket>();

        // Initialise la couleur actuelle avec la couleur par d�faut
        currentColor = defaultColor;
    }

    void Update()
    {
        // Acc�der au niveau sonore en d�cibels via l'instance de UdpSocket
        float decibels = udpSocket.soundLevel;

        // Changer la couleur de la balle en fonction du niveau sonore
        if (decibels > decibelThreshold)
        {
            ChangeSphereColor(highDecibelColor);
        }
        else
        {
            ChangeSphereColor(defaultColor);
        }
    }

    void ChangeSphereColor(Color newColor)
    {
        // V�rifier si la couleur a chang�
        if (newColor != currentColor)
        {
            // Afficher un log si la couleur change
            Debug.Log("La couleur de la sph�re a chang� !");
            currentColor = newColor;
        }

        // Appliquer la nouvelle couleur � la sph�re
        Sphere.GetComponent<Renderer>().material.color = newColor;
    }
}
