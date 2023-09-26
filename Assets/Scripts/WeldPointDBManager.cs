using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WeldPointDBManager : MonoBehaviour
{
    [SerializeField] private WeldPointDataBase WeldPointDataBase;

    public void AddWeldPointData(WeldPoint WeldPoint)
    {
        WeldPointDataBase.WeldPointBaseList.Add(WeldPoint);
    }
    // Start is called before the first frame update
    void Start()
    {
  
       /*
        int count = WeldPointDataBase.WeldPointBaseList.Count; // WeldPointListÇÃóvëfêî

       for (int i = 0; i < count; i++)
         {
             WeldPoint WeldP = ScriptableObject.CreateInstance("WeldPoint") as WeldPoint;
             WeldP = WeldPointDataBase.WeldPointBaseList[i];
             Debug.Log(WeldP.Weld_Name + " " + WeldP.id);
         }
       */
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
