using UnityEngine;
using System.Collections;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using SimpleJSON;

public class UdpSocketV2 : MonoBehaviour
{
    [HideInInspector] public bool isTxStarted = false;
    [SerializeField] string IP = "127.0.0.1"; // localhost
    [SerializeField] int rxPort = 8000; // port to receive data from Python on
    [SerializeField] int txPort = 8001; // port to send data to Python on

    int i = 0;

    // Create necessary UdpClient objects
    UdpClient client;
    IPEndPoint remoteEndPoint;
    Thread receiveThread; // Receiving Thread

    IEnumerator SendDataCoroutine()
    {
        while (true)
        {
            // SendData("Sent from Unity: " + i.ToString());
            i++;
            yield return new WaitForSeconds(1f);
        }
    }

    void Awake()
    {
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(IP), txPort);
        client = new UdpClient(rxPort);
        receiveThread = new Thread(new ThreadStart(Start));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        Debug.Log("UDP Comms Initialized");

        StartCoroutine(SendDataCoroutine());
    }

    private void Start()
    {
        StartCoroutine(ReceiveData());
    }

    private IEnumerator ReceiveData()
    {
        while (true)
        {
            byte[] data = client.Receive(ref remoteEndPoint);
            string message = Encoding.UTF8.GetString(data);

            // ProcessInput(message);
            // Ajoutez le code pour traiter les donnÃƒÂ©es de mouvement reÃƒÂ§ues depuis Python
            ProcessMovementData(message);

            yield return null;
        }
    }

    private void ProcessMovementData(string input)
    {
        Debug.Log("Received JSON: " + input);

        try
        {
            JSONNode rootNode = JSON.Parse(input);

            if (rootNode.HasKey("'leftArmAngle'") && rootNode.HasKey("'rightArmAngle'") && rootNode.HasKey("'leftShoulderAngle'") && rootNode.HasKey("'rightShoulderAngle'"))
            {
                float leftArmAngle = rootNode["'leftArmAngle'"].AsFloat;
                float rightArmAngle = rootNode["'rightArmAngle'"].AsFloat;
                float leftShoulderAngle = rootNode["'leftShoulderAngle'"].AsFloat;
                float rightShoulderAngle = rootNode["'rightShoulderAngle'"].AsFloat;

                Debug.Log("Left Arm Angle: " + leftArmAngle);
                Debug.Log("Right Arm Angle: " + rightArmAngle);
                Debug.Log("Left Shoulder Angle: " + leftShoulderAngle);
                Debug.Log("Right Shoulder Angle: " + rightShoulderAngle);

                ApplyRotations(leftArmAngle, rightArmAngle, leftShoulderAngle, rightShoulderAngle);
            }
            else
            {
                Debug.LogError("Missing keys in the JSON.");
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error parsing JSON: " + e.Message);
        }
    }

    private void ApplyRotations(float leftArmAngle, float rightArmAngle, float leftShoulderAngle,float rightShoulderAngle)
    {
        // Appliquer les rotations aux parties du bras gauche
        GameObject upperArmLeft = GameObject.Find("Left_UpperArm");
        GameObject lowerArmLeft = GameObject.Find("Left_LowerArm");

        // Appliquer les rotations aux parties du bras droit
        GameObject upperArmRight = GameObject.Find("Right_UpperArm");
        GameObject lowerArmRight = GameObject.Find("Right_LowerArm");

        // Appliquer les rotations aux parties des épaules
        GameObject leftShoulder = GameObject.Find("Left_Shoulder");
        GameObject rightShoulder = GameObject.Find("Right_Shoulder");

        // Vérifier si les GameObjects sont trouvés
        if (upperArmLeft != null && lowerArmLeft != null &&
            upperArmRight != null && lowerArmRight != null &&
            leftShoulder != null && rightShoulder != null)
        {
            Debug.Log("All GameObjects found. Applying rotations...");

            // Inverser l'axe Z pour les parties du bras
            ApplyRotation(upperArmLeft, leftArmAngle, Vector3.forward);
            ApplyRotation(lowerArmLeft, -leftArmAngle, Vector3.forward);

            ApplyRotation(upperArmRight, rightArmAngle, Vector3.forward);
            ApplyRotation(lowerArmRight, -rightArmAngle, Vector3.forward);

        }
        else
        {
            Debug.LogError("Some GameObjects not found. Check the names in the script.");
        }
    }

    private void ApplyRotation(GameObject bodyPart, float rotationAngle, Vector3 rotationAxis)
    {
        if (bodyPart != null)
        {
            Quaternion rotation = Quaternion.AngleAxis(rotationAngle, rotationAxis);
            bodyPart.transform.localRotation = rotation;
        }
        else
        {
            Debug.LogError("Partie du corps non trouvé.");
        }
    }
    void OnDisable()
    {
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }

        if (client != null)
        {
            client.Close();
        }
    }
}
