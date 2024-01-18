using UnityEngine;
using System.Collections;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class UdpSocket : MonoBehaviour
{
    [HideInInspector] public bool isTxStarted = false;
    public float wristDistance = 0.0f;
    public float soundLevel = 0.0f;
    [SerializeField] string IP = "127.0.0.1"; // local host
    [SerializeField] int rxPort = 8000; // port to receive data from Python on
    [SerializeField] int txPort = 8001; // port to send data to Python on

    // Create necessary UdpClient objects
    UdpClient client;
    IPEndPoint remoteEndPoint;
    Thread receiveThread; // Receiving Thread

    int i = 0;
    IEnumerator SendDataCoroutine()
    {
        while (true)
        {
            SendData("Sent from Unity: " + i.ToString());
            i++;
            yield return new WaitForSeconds(1f);
        }
    }

    public void SendData(string message) // Use to send data to Python
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(message);
            client.Send(data, data.Length, remoteEndPoint);
        }
        catch (Exception err)
        {
            print(err.ToString());
        }
    }

    void Awake()
    {
        // Create remote endpoint (to Matlab) 
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(IP), txPort);

        // Create local client
        client = new UdpClient(rxPort);

        // local endpoint define (where messages are received)
        // Create a new thread for reception of incoming messages
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        // Initialize (seen in comments window)
        print("UDP Comms Initialised");

        StartCoroutine(SendDataCoroutine());
    }

    // Receive data, update packets received
    private void ReceiveData()
    {
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = client.Receive(ref anyIP);
                string text = Encoding.UTF8.GetString(data);
                print(">> " + text);
                ProcessInput(text);
            }
            catch (Exception err)
            {
                print(err.ToString());
            }
        }
    }

private void ProcessInput(string input)
{
    // Analyser l'entrée reçue pour extraire la distance du poignet
    if (input.Contains("distance poignet :"))
    {
        string[] parts = input.Split(':');
        if (parts.Length > 1)
        {
            string distanceStr = parts[1].Trim(); // Supprime les espaces autour du nombre

            // Utiliser la culture "en-US" pour garantir que la conversion fonctionne avec un point décimal
            if (float.TryParse(distanceStr, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float distance))
            {
                wristDistance = distance;
            }
            else
            {
                Debug.LogError("Échec de la conversion de la distance du poignet en float. Chaîne invalide: " + distanceStr);
            }
        }
    }
    
    if (input.StartsWith("niveau_sonore_dB:"))
    {
        string decibelsStr = input.Substring("niveau_sonore_dB:".Length).Trim(); // Supprime le préfixe et les espaces autour du nombre
        Debug.Log("Chaîne de niveau sonore reçue: " + decibelsStr);
        if (float.TryParse(decibelsStr, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float decibels))
        {
            soundLevel = decibels;
        }
        else
        {
            Debug.LogError("Échec de la conversion du niveau sonore en float. Chaîne invalide: " + decibelsStr);
        }
    }
            
    if (!isTxStarted)
    {
        isTxStarted = true;
    }
}

    //Prevent crashes - close clients and threads properly!
    void OnDisable()
    {
        if (receiveThread != null)
            receiveThread.Abort();

        client.Close();
    }

}