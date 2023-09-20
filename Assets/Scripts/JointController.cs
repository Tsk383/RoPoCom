using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.Providers.LinearAlgebra;


namespace InverseKinematics
{
    public class JointController : MonoBehaviour
    {
        //robot
        private GameObject[] joint = new GameObject[6];
        private float[] angle = new float[6];
        // アームの寸法
        private Vector3[] dim = new Vector3[6];
        // ローカル座標軸のワールド座標
        private Vector3[] point = new Vector3[7];
        // 回転軸の向き
        private Vector3[] axis = new Vector3[6];
        // 各軸の親基準ローカル回転クオータニオン
        private Quaternion[] rotation = new Quaternion[6];
        // 各軸のワールド回転クオータニオン
        private Quaternion[] wRotation = new Quaternion[6];

        private Vector3 pos; //目標位置
        private Vector3 rot; //目標姿勢
        private float lambda = 0.1f; // 収束計算の調整用

        //WeldPointの座標取得用
        private GameObject targetObject;

        //リーチ可否の判定結果記述用
        private GameObject ReachResult; //= GameObject.Find("ReachResult");


        //UI
        private GameObject[] slider = new GameObject[6];
        private float[] sliderVal = new float[6];
        private float[] prevSliderVal = new float[6];
        private GameObject[] angText = new GameObject[6];
        private GameObject[] posText = new GameObject[6];
        private float[] prevAngle = new float[6];
        private float[] minAngle = new float[6];
        private float[] maxAngle = new float[6];

        //WeldPosition1のInpufield用
        private GameObject[] P1_posText = new GameObject[3];
        private GameObject[] P1_angText = new GameObject[3];

        //InputField型の変数を宣言対象のInputFieldのオブジェクトをアタッチする
        [SerializeField] private InputField P1_InputField;
        //Vector3 WeldPointPosition = gameObject.transform.position;

        void CalcIK()
        {
            int count = 0;
            bool outOfLimit = false;
            for (int i = 0; i < 100; i++) // 繰り返し計算
            {
                count = i;
                ForwardKinematics(); // 手先の位置・姿勢を求める
                var err = CalcErr(); // 目標値からの隔たり
                float err_norm = (float)err.L2Norm(); //隔たりの絶対値
                if (err_norm < 1E-3) // 繰り返しを抜ける
                {
                    for (int ii = 0; ii < joint.Length; ii++)
                    {
                        if (angle[ii] < minAngle[ii] || angle[ii] > maxAngle[ii])
                        {
                            outOfLimit = true;
                            break;
                        }
                    }
                    break;
                }
                var J = CalcJacobian(); // ヤコビ行列を求める
                // 角度を修正
                var dAngle = lambda * J.PseudoInverse() * err;
                for (int ii = 0; ii < joint.Length; ii++)
                {
                    angle[ii] += dAngle[ii, 0] * Mathf.Rad2Deg;
                }
            }
            if (count == 99 || outOfLimit) // 収束せず or 角度オーバー
            {
                for (int i = 0; i < joint.Length; i++)
                {
                    slider[i].GetComponent<Slider>().value = prevSliderVal[i];
                    angle[i] = prevAngle[i];

                    //到達不可能な場合はデバッグでエラーを表示する
                    ReachResult.GetComponent<TextMeshProUGUI>().text = "NG";
                    //Debug.Log("ERROR");
                }
            }
            else      // ロボットの関節角度を更新
            {
                for (int i = 0; i < joint.Length; i++)
                {
                    rotation[i] = Quaternion.AngleAxis(angle[i], axis[i]);
                    joint[i].transform.localRotation = rotation[i];
                    prevSliderVal[i] = sliderVal[i];
                    prevAngle[i] = angle[i];
                    posText[i].GetComponent<TextMeshProUGUI>().text = sliderVal[i].ToString("f2");
                    angText[i].GetComponent<TextMeshProUGUI>().text = angle[i].ToString("f2");
                }
                ReachResult.GetComponent<TextMeshProUGUI>().text = "OK";
            }
        }
        void ForwardKinematics()
        {
            point[0] = new Vector3(0f,0f,0f); 
            wRotation[0] = Quaternion.AngleAxis(angle[0],axis[0]);
            for (int i = 1; i < joint.Length; i++)
            {
                point[i] = wRotation[i - 1] * dim[i - 1] + point[i - 1];
                rotation[i] = Quaternion.AngleAxis(angle[i], axis[i]);
                wRotation[i] = wRotation[i - 1] * rotation[i];
            }
            point[joint.Length] = wRotation[joint.Length - 1] * dim[joint.Length - 1] + point[joint.Length - 1];
        }

        DenseMatrix CalcErr()
        { 
            // 位置誤差
            Vector3 perr = pos - point[6];
            // 姿勢誤差
            Quaternion rerr = Quaternion.Euler(rot) * Quaternion.Inverse(wRotation[5]);
            // xyz周りの回転に変換
            Vector3 rerrVal = new Vector3(rerr.eulerAngles.x,rerr.eulerAngles.y,rerr.eulerAngles.z);
            if (rerrVal.x > 180f) rerrVal.x -= 360f;
            if (rerrVal.y > 180f) rerrVal.y -= 360f;
            if (rerrVal.z > 180f) rerrVal.z -= 360f;
            var err = DenseMatrix.OfArray( new float[,] 
            {
                { perr.x },
                { perr.y },
                { perr.z },
                { rerrVal.x * Mathf.Deg2Rad},
                { rerrVal.y * Mathf.Deg2Rad},
                { rerrVal.z * Mathf.Deg2Rad}
            });
            return err;
        }

        DenseMatrix CalcJacobian()
        {
            Vector3 w0 = wRotation[0] * axis[0];
            Vector3 w1 = wRotation[1] * axis[1];
            Vector3 w2 = wRotation[2] * axis[2];
            Vector3 w3 = wRotation[3] * axis[3];
            Vector3 w4 = wRotation[4] * axis[4];
            Vector3 w5 = wRotation[5] * axis[5];
            Vector3 p0 = Vector3.Cross(w0, point[6] - point[0]);
            Vector3 p1 = Vector3.Cross(w1, point[6] - point[1]);
            Vector3 p2 = Vector3.Cross(w2, point[6] - point[2]);
            Vector3 p3 = Vector3.Cross(w3, point[6] - point[3]);
            Vector3 p4 = Vector3.Cross(w4, point[6] - point[4]);
            Vector3 p5 = Vector3.Cross(w5, point[6] - point[5]);
            var J = DenseMatrix.OfArray(new float[,]
            {
                { p0.x, p1.x, p2.x, p3.x, p4.x, p5.x },
                { p0.y, p1.y, p2.y, p3.y, p4.y, p5.y },
                { p0.z, p1.z, p2.z, p3.z, p4.z, p5.z },
                { w0.x, w1.x, w2.x, w3.x, w4.x, w5.x },
                { w0.y, w1.y, w2.y, w3.y, w4.y, w5.y },
                { w0.z, w1.z, w2.z, w3.z, w4.z, w5.z }
            });
            return J;
        }

        // Start is called before the first frame update
        void Start()
        {
            //robot
            for(int i=0;i<joint.Length;i++)
            {
                joint[i] = GameObject.Find("Joint_" + i.ToString());
            }

            //UI settings
            for(int i = 0; i < joint.Length; i++)
            {
                slider[i] = GameObject.Find("Slider_" + i.ToString());
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
                posText[i] = GameObject.Find("Ref_"+i.ToString());
                angText[i] = GameObject.Find("Ang_" + i.ToString());
            }
            // イニシャル姿勢での各アームの寸法
            dim[0] = new Vector3( 0f,2f,0f);
            dim[1] = new Vector3( 4f,0f,0f);
            dim[2] = new Vector3( 1f,0f,0f);
            dim[3] = new Vector3( 2f,0f,0f);
            dim[4] = new Vector3( 1f,0f,0f);
            dim[5] = new Vector3( 2f,0f,0f);

            // 各回転軸の方向
            axis[0] = new Vector3( 0f,1f,0f);
            axis[1] = new Vector3( 0f,0f,1f);
            axis[2] = new Vector3( 0f,0f,1f);
            axis[3] = new Vector3( 1f,0f,0f);
            axis[4] = new Vector3( 0f,0f,1f);
            axis[5] = new Vector3( 1f,0f,0f);

            // イニシャル姿勢での回転角
            angle[0] = 0f;
            angle[1] = 90f;
            angle[2] = -90f;
            angle[3] = 0f;
            angle[4] = 0f;
            angle[5] = 0f;
            for (int i = 0; i < joint.Length; i++)
            {
                minAngle[i] = -150f;
                maxAngle[i] = 150f;
            }
            // オブジェクトを検索
            targetObject = GameObject.Find("WeldPoint_0");

            //リーチ到達不可の結果用オブジェクトの検索
            ReachResult = GameObject.Find("P1_ReachResult");

            //WeldPointの座標をinputFieldにインプット
            P1_posText[0] = GameObject.Find("P1_Input_Px");
           // P1_InputField.text= InputField("P1_InputPx");
        //private GameObject[] P1_angText = new GameObject[3];

        // オブジェクトのワールド座標を取得
            Vector3 worldPosition1 = targetObject.transform.position;
            Vector3 worldPosition2 = targetObject.transform.localEulerAngles;

            // ワールド座標を表示
            //Debug.Log(worldPosition1);
            //Debug.Log(worldPosition2);

            sliderVal[0] = worldPosition1.x;
            sliderVal[1] = worldPosition1.y;
            sliderVal[2] = worldPosition1.z;
            sliderVal[3] = worldPosition2.x;
            sliderVal[4] = worldPosition2.y;
            sliderVal[5] = worldPosition2.z;

            slider[0].GetComponent<Slider>().value = worldPosition1.x;
            slider[1].GetComponent<Slider>().value = worldPosition1.y;
            slider[2].GetComponent<Slider>().value = worldPosition1.z;
            slider[3].GetComponent<Slider>().value = worldPosition2.x;
            slider[4].GetComponent<Slider>().value = worldPosition2.y;
            slider[5].GetComponent<Slider>().value = worldPosition2.z;

            /*Input fieldにPxの値を代入*/
            //  InputPx = slider[0].ToStrings();
            //   ReachResult.GetComponent<TextMeshProUGUI>().text = "NG";
            //  private GameObject[] P1_posText = new GameObject[3];
            //private GameObject[] P1_angText = new GameObject[3];

        }

        // Update is called once per frame
        void Update()
        {
            for (int i = 0; i < joint.Length; i++)
            {
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
            }
            // オブジェクトのワールド座標を取得
         /*   Vector3 worldPosition1 = targetObject.transform.position.x;
            Vector3 worldPosition2 = targetObject.transform.localEulerAngles;
            sliderVal[0] = worldPosition1.x;
            sliderVal[1] = worldPosition1.y;
            sliderVal[2] = worldPosition1.z;
            sliderVal[3] = worldPosition2.x;
            sliderVal[4] = worldPosition2.y;
            sliderVal[5] = worldPosition2.z;*/
          
            pos.x = targetObject.transform.position.x;
            pos.y = targetObject.transform.position.y;
            pos.z = targetObject.transform.position.z;
            rot.x = targetObject.transform.localEulerAngles.x;
            rot.y = targetObject.transform.localEulerAngles.y;
            rot.z = targetObject.transform.localEulerAngles.z;
            /*
            pos.x = sliderVal[0];
            pos.y = sliderVal[1];
            pos.z = sliderVal[2];
            rot.x = sliderVal[3];
            rot.y = sliderVal[4];
            rot.z = sliderVal[5];
            */

            // IK
            CalcIK();
        }
    }
}
