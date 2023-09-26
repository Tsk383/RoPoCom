using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
[SerializeField]
public class WeldPointDataBase : ScriptableObject
{
    public List<WeldPoint> WeldPointBaseList = new List<WeldPoint>();
}
