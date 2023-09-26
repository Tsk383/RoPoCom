using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
[SerializeField]
public class WeldPoint : ScriptableObject
{
    public int id;                                       //ID
    public string Weld_Name;                             //–¼Ì
    public float Weld_Px;                                //—nÚ‰ÓŠ‚ÌxÀ•W
    public float Weld_Py;                                //—nÚ‰ÓŠ‚ÌyÀ•W
    public float Weld_Pz;                                //—nÚ‰ÓŠ‚ÌzÀ•W
    public float Weld_Rx;                                //—nÚ‰ÓŠ‚ÌxŠp“x
    public float Weld_Ry;                                //—nÚ‰ÓŠ‚ÌyŠp“x
    public float Weld_Rz;                                //—nÚ‰ÓŠ‚ÌzŠp“x
    public string ReachCheck;                            //ƒƒ{ƒbƒg‚Ì“’B‰Â”Û”»’èŒ‹‰Ê
}