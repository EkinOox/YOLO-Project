using UnityEngine;

public class ChangeColor : MonoBehaviour
{
    public UdpSocket udpSocket;
    public GameObject Sphere;
    public float decibelThreshold = 70.0f;
    public Color highDecibelColor = Color.red;
    public Color defaultColor = Color.white;

    private Color currentColor;  // Stocke la couleur actuelle pour vérifier les changements

    void Start()
    {
        // Assurez-vous d'ajouter un composant UdpSocket à l'objet qui a ce script ChangeColor
        udpSocket = GetComponent<UdpSocket>();

        // Initialise la couleur actuelle avec la couleur par défaut
        currentColor = defaultColor;
    }

    void Update()
    {
        // Accéder au niveau sonore en décibels via l'instance de UdpSocket
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
        // Vérifier si la couleur a changé
        if (newColor != currentColor)
        {
            // Afficher un log si la couleur change
            Debug.Log("La couleur de la sphère a changé !");
            currentColor = newColor;
        }

        // Appliquer la nouvelle couleur à la sphère
        Sphere.GetComponent<Renderer>().material.color = newColor;
    }
}
