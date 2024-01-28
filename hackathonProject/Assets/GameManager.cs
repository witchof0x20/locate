using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{

    public static GameManager Instance;

    public Dictionary<Vector3Int, GameObject> voxels;
    public Queue<Vector3Int> voxelQueue;

    private static float MILLIMETER_CONVERSION = 1000;

    private static float VOXEL_GRID_RESOLUTION = 0.1f;

    private static float MAX_VOXELS = 40000;

    private GameObject spheresContainer;

    private Vector3 cameraOffset;

    void Awake() {
        Instance = this;
        spheresContainer = GameObject.Find("SpheresContainer");
        voxels = new Dictionary<Vector3Int, GameObject>();
        voxelQueue = new Queue<Vector3Int>();
        cameraOffset = new Vector3(0, 0, 0);
    }

    void Update() {
    }

    Vector3 Rotate2D(Vector3 v, float angle) {
        float sinA = Mathf.Sin(angle);
        float cosA = Mathf.Cos(angle);

        return new Vector3(v.x*cosA + v.z*sinA, v.y, v.z*cosA + v.x*sinA);
    }

    void DrawPoint(Vector3 v) {
        //Debug.Log("SUCCESS!!");

        Vector3Int lookupVec = Vector3Int.FloorToInt(new Vector3(Mathf.Floor(v.x/VOXEL_GRID_RESOLUTION), 
            Mathf.Floor(v.y/VOXEL_GRID_RESOLUTION),
            Mathf.Floor(v.z/VOXEL_GRID_RESOLUTION)));
        if (!voxels.ContainsKey(lookupVec)) {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.transform.SetParent(spheresContainer.transform, true);
            Vector3 cubeVec = new Vector3(Mathf.Floor(v.x/VOXEL_GRID_RESOLUTION)*VOXEL_GRID_RESOLUTION, 
                Mathf.Floor(v.y/VOXEL_GRID_RESOLUTION)*VOXEL_GRID_RESOLUTION, 
                Mathf.Floor(v.z/VOXEL_GRID_RESOLUTION)*VOXEL_GRID_RESOLUTION);
            cube.transform.localPosition = cubeVec;
            cube.transform.localScale = new Vector3(VOXEL_GRID_RESOLUTION, VOXEL_GRID_RESOLUTION, VOXEL_GRID_RESOLUTION);
            voxels.Add(lookupVec, cube);
            voxelQueue.Enqueue(lookupVec);
            if (voxelQueue.Count > MAX_VOXELS) {
                Vector3Int destroyPosition = voxelQueue.Dequeue();
                GameObject destroyObject = voxels[destroyPosition];
                voxels.Remove(destroyPosition);
                Destroy(destroyObject);
            }
        }
            
    }

    public void AddPoint(float lAngle, Vector3 cameraEuler, float distance) {
        Vector3 position = Vector3.forward;
        position = Rotate2D(position, lAngle*Mathf.PI/180);
        position = Quaternion.Euler(cameraEuler)*position;
        position *= distance/MILLIMETER_CONVERSION;
        

        //position = Rotate2D(position, lAngle*Mathf.PI/180);
        //Debug.Log(position);
        DrawPoint(position);
    }

}
