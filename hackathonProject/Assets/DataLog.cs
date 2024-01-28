using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DataLog
{
    public float curDegree;

    public float curXRotate;
    public float curYRotate;
    public float curZRotate;

    public Vector3 camera;
    public Vector3 collision;
    public float distance;

    public byte[] collisionData = new byte[18];
    public  byte[] emptyEuler = { 00000000, 00000000, 00000000, 00000000 };

    public DataLog(float degree, float xRotate, float yRotate, Vector3 cameraOrigin, Vector3 collisionPoint)
    {
        curDegree = degree;
        curXRotate = xRotate;
        curYRotate = yRotate;
        curZRotate = 0;
        camera = cameraOrigin;
        collision = collisionPoint;
        distance = vectorsToFloatDistance(collision, camera);

        byte[] spin = spinToAngle(curDegree);
        byte[] xEuler = eulerToAngle(curXRotate);

        populateData(spinToAngle(curDegree), eulerToAngle(curXRotate), eulerToAngle(curYRotate), vectorsToByteDistance(collision, camera));
    }

    public DataLog(byte[] byteData)
    {
        byte[] spin = { byteData[0], byteData[1] };
        byte[] byteDistance = { byteData[2], byteData[3], byteData[4], byteData[5] };
        byte[] xEuler = { byteData[6], byteData[7], byteData[8], byteData[9] };
        byte[] yEuler = { byteData[10], byteData[11], byteData[12], byteData[13] };
        byte[] zEuler = { byteData[14], byteData[15], byteData[16], byteData[17] };

        curDegree = (float)System.BitConverter.ToUInt16(spin, 0);
        distance = System.BitConverter.ToSingle(byteDistance, 0);
        curXRotate = System.BitConverter.ToSingle(xEuler, 0);
        curYRotate = System.BitConverter.ToSingle(yEuler, 0);
        curZRotate = System.BitConverter.ToSingle(zEuler, 0);
    }

    public byte[] spinToAngle(float angle)
    {
        ushort flooredAngle = (ushort) angle;
        byte[] bytes = System.BitConverter.GetBytes(flooredAngle);
        return bytes;
    }

    public byte[] eulerToAngle(float angle)
    {
        byte[] bytes = System.BitConverter.GetBytes(angle);
        return bytes;
    }

    public byte[] vectorsToByteDistance(Vector3 collision, Vector3 camera)
    {
        Vector3 diffVec = new Vector3(collision.x - camera.x, collision.y - camera.y, collision.z - camera.z);
        float distance = Mathf.Sqrt(diffVec.x * diffVec.x + diffVec.y * diffVec.y + diffVec.z * diffVec.z);
        byte[] bytes = System.BitConverter.GetBytes(distance);
        return bytes;
    }

    public float vectorsToFloatDistance(Vector3 collision, Vector3 camera)
    {
        Vector3 diffVec = new Vector3(collision.x - camera.x, collision.y - camera.y, collision.z - camera.z);
        float fDistance = Mathf.Sqrt(diffVec.x * diffVec.x + diffVec.y * diffVec.y + diffVec.z * diffVec.z);
        return fDistance;
    }

    public void populateData(byte[] spin, byte[] xEuler, byte[] yEuler, byte[] distance)
    {
        System.Array.Copy(spin, spin.GetLowerBound(0), collisionData, collisionData.GetLowerBound(0), 2);
        System.Array.Copy(distance, distance.GetLowerBound(0), collisionData, collisionData.GetLowerBound(0) + 2, 4);
        System.Array.Copy(xEuler, xEuler.GetLowerBound(0), collisionData, collisionData.GetLowerBound(0) + 6, 4);
        System.Array.Copy(yEuler, yEuler.GetLowerBound(0), collisionData, collisionData.GetLowerBound(0) + 10, 4);
        System.Array.Copy(emptyEuler, emptyEuler.GetLowerBound(0), collisionData, collisionData.GetLowerBound(0) + 14, 4);

    }

    public static byte[] composeData(byte[] spin, byte[] xEuler, byte[] yEuler, byte[] distance)
    {
        byte[] emptyBytes = { 00000000, 00000000, 00000000, 00000000 };
        byte[] composedBytes = new byte[18];

        System.Array.Copy(spin, spin.GetLowerBound(0), composedBytes, composedBytes.GetLowerBound(0), 2);
        System.Array.Copy(distance, distance.GetLowerBound(0), composedBytes, composedBytes.GetLowerBound(0) + 2, 4);
        System.Array.Copy(xEuler, xEuler.GetLowerBound(0), composedBytes, composedBytes.GetLowerBound(0) + 6, 4);
        System.Array.Copy(yEuler, yEuler.GetLowerBound(0), composedBytes, composedBytes.GetLowerBound(0) + 10, 4);
        System.Array.Copy(emptyBytes, emptyBytes.GetLowerBound(0), composedBytes, composedBytes.GetLowerBound(0) + 14, 4);

        return composedBytes;
    }

   
}
