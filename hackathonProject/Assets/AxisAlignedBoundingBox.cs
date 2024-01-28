using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AxisAlignedBoundingBox : MonoBehaviour
{

    public Vector3 pos;
    public Vector3 size;

    public AxisAlignedBoundingBox() {
        pos = new Vector3(0, 0, 0);
        size = new Vector3(0, 0, 0);
    }
}
