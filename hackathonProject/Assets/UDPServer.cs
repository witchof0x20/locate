using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

public class UDPServer : MonoBehaviour
{
    private UdpClient udpServer;
    private IPEndPoint remoteEndPoint;

    private Queue<DataLog> dataLogs;

    private bool useAsyncEuler;
    private Vector3 asyncEuler;

    private static int PORT = 5555;


    // Start is called before the first frame update
    void Awake() {
        udpServer = new UdpClient(PORT);
        remoteEndPoint = new IPEndPoint(IPAddress.Any, PORT);
        udpServer.BeginReceive(ReceiveData, null);
        useAsyncEuler = false;

        dataLogs = new Queue<DataLog>();
    }

    void Update() {

        if (dataLogs.Count > 0) {
            for (int i = 0; i < Mathf.Min(dataLogs.Count, 10); i++) {
                DataLog lastDataLog = dataLogs.Dequeue();
                if (lastDataLog.curDegree == 5555) {
                    if (useAsyncEuler==false) {
                        Debug.Log("Switching Modes to Async Euler!");
                    }
                    asyncEuler = new Vector3(lastDataLog.curXRotate, lastDataLog.curYRotate, lastDataLog.curZRotate) * 180/Mathf.PI;
                    useAsyncEuler = true;
                } else {
                    Vector3 eulerAngles = useAsyncEuler ? asyncEuler : new Vector3(lastDataLog.curXRotate, lastDataLog.curYRotate, lastDataLog.curZRotate);
                    //Debug.Log(eulerAngles);
                    GameManager.Instance.AddPoint(lastDataLog.curDegree, eulerAngles, lastDataLog.distance);
                }
            }
            //Debug.Log("SUCCESS!");
            
        }
    }

    private void ReceiveData(IAsyncResult result)
    {
        //Debug.Log("Got Bytes?");
        byte[] receivedBytes = udpServer.EndReceive(result, ref remoteEndPoint);
        if (receivedBytes.Length == 18) {
            //Debug.Log("Do Got Bytes!");
            dataLogs.Enqueue(new DataLog(receivedBytes));
            //GameManager.Instance.AddPoint(newPoint.curDegree, eulerAngles, newPoint.distance*1000);
        }
        // Process the received data
        // Continue receiving data asynchronously
        udpServer.BeginReceive(ReceiveData, null);
    }
}
