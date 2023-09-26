using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
[SerializeField]
public class WeldPoint : ScriptableObject
{
    public int id;                                       //ID
    public string Weld_Name;                             //����
    public float Weld_Px;                                //�n�ډӏ���x���W
    public float Weld_Py;                                //�n�ډӏ���y���W
    public float Weld_Pz;                                //�n�ډӏ���z���W
    public float Weld_Rx;                                //�n�ډӏ���x�p�x
    public float Weld_Ry;                                //�n�ډӏ���y�p�x
    public float Weld_Rz;                                //�n�ډӏ���z�p�x
    public string ReachCheck;                            //���{�b�g�̓��B�۔��茋��
}