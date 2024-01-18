using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class server : MonoBehaviour
{
    public LineRenderer leftElbowLineRenderer;
    public LineRenderer rightElbowLineRenderer;

    private TcpClient tcpClient;
    private Thread tcpClientThread;
    private bool shouldRun = true;

    void Start()
    {
        tcpClientThread = new Thread(new ThreadStart(ConnectToPythonServer));
        tcpClientThread.Start();
    }

    private void ConnectToPythonServer()
    {
        try
        {
            tcpClient = new TcpClient("127.0.0.1", 8080);
            SendMessageToPython("StartYolo"); // Envoyer le message de démarrage
            ListenForData();
        }
        catch (SocketException e)
        {
            Debug.LogError("Error connecting to Python server: " + e.Message);
        }
    }

    private void ListenForData()
    {
        while (shouldRun)
        {
            using (NetworkStream ns = tcpClient.GetStream())
            {
                byte[] data = new byte[1024];
                int bytesRead = ns.Read(data, 0, data.Length);

                if (bytesRead > 0)
                {
                    string receivedData = Encoding.UTF8.GetString(data, 0, bytesRead);
                    Debug.Log("Received data from Python: " + receivedData);

                    // Convertir les données reçues en un tableau de positions
                    string[] positionsArray = receivedData.Split(';');
                    Vector3 leftElbowPosition = ParsePosition(positionsArray[0]);
                    Vector3 rightElbowPosition = ParsePosition(positionsArray[1]);

                    // Mettre à jour les positions des Line Renderers
                    UpdateLineRenderer(leftElbowLineRenderer, leftElbowPosition);
                    UpdateLineRenderer(rightElbowLineRenderer, rightElbowPosition);
                }
            }
        }
    }

    private Vector3 ParsePosition(string positionString)
    {
        string[] components = positionString.Split(',');
        float x = float.Parse(components[0]);
        float y = float.Parse(components[1]);
        float z = float.Parse(components[2]);
        return new Vector3(x, y, z);
    }

    private void UpdateLineRenderer(LineRenderer lineRenderer, Vector3 position)
    {
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, position);
        lineRenderer.SetPosition(1, position + Vector3.up * 2f); // Ajustez la hauteur du plot comme nécessaire
    }

    void OnDestroy()
    {
        shouldRun = false;
        if (tcpClient != null)
        {
            tcpClient.Close();
        }
    }

    private void SendMessageToPython(string message)
    {
        if (tcpClient != null && tcpClient.Connected)
        {
            NetworkStream ns = tcpClient.GetStream();
            byte[] data = Encoding.UTF8.GetBytes(message);
            ns.Write(data, 0, data.Length);
        }
        else
        {
            Debug.LogWarning("Not connected to Python server. Message not sent.");
        }
    }
}
