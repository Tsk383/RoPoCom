using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.Providers.LinearAlgebra;

using System.IO;


namespace InverseKinematics
{
    public class JointController : MonoBehaviour
    {

        private const string CsvPath = "Assets/Resources/WeldPoint.csv";
        TextAsset csvFile; // CSV�t�@�C��
        List<string[]> csvDatas = new List<string[]>(); // CSV�̒��g�����郊�X�g;
        

        //robot
        private GameObject[] joint = new GameObject[6];
        private float[] angle = new float[6];
        // �A�[���̐��@
        private Vector3[] dim = new Vector3[6];
        // ���[�J�����W���̃��[���h���W
        private Vector3[] point = new Vector3[7];
        // ��]���̌���
        private Vector3[] axis = new Vector3[6];
        // �e���̐e����[�J����]�N�I�[�^�j�I��
        private Quaternion[] rotation = new Quaternion[6];
        // �e���̃��[���h��]�N�I�[�^�j�I��
        private Quaternion[] wRotation = new Quaternion[6];

        private Vector3 pos; //�ڕW�ʒu
        private Vector3 rot; //�ڕW�p��
        private float lambda = 0.1f; // �����v�Z�̒����p

        //WeldPoint�̍��W�擾�p
        private GameObject targetObject;

        //WeldPoint��ID���ʗp
        private int WeldPoint_ID = 0;

        //WeldPoint�̃��X�g�J�E���g�p
        private GameObject[] WeldPointCount;

        //���[�`�ۂ̔��茋�ʋL�q�p
        private GameObject ReachResult;

        //UI
        private GameObject[] slider = new GameObject[6];
        private float[] sliderVal = new float[6];
        private float[] prevSliderVal = new float[6];
        private GameObject[] angText = new GameObject[6];
        private GameObject[] posText = new GameObject[6];
        private float[] prevAngle = new float[6];
        private float[] minAngle = new float[6];
        private float[] maxAngle = new float[6];

        //WeldPoint��Inpufield�p
        private TMP_InputField WeldPointInput_pos_x;
        private TMP_InputField WeldPointInput_pos_y;
        private TMP_InputField WeldPointInput_pos_z;
        private TMP_InputField WeldPointInput_ang_x;
        private TMP_InputField WeldPointInput_ang_y;
        private TMP_InputField WeldPointInput_ang_z;

        // Updata�֐����̃t���O�p
        bool isCalledOnce = false;


        void CalcIK()
        {
            int count = 0;
            bool outOfLimit = false;
            for (int i = 0; i < 100; i++) // �J��Ԃ��v�Z
            {
                count = i;
                ForwardKinematics(); // ���̈ʒu�E�p�������߂�
                var err = CalcErr(); // �ڕW�l����̊u����
                float err_norm = (float)err.L2Norm(); //�u����̐�Βl
                if (err_norm < 1E-3) // �J��Ԃ��𔲂���
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
                var J = CalcJacobian(); // ���R�r�s������߂�
                // �p�x���C��
                var dAngle = lambda * J.PseudoInverse() * err;
                for (int ii = 0; ii < joint.Length; ii++)
                {
                    angle[ii] += dAngle[ii, 0] * Mathf.Rad2Deg;
                }
            }
            if (count == 99 || outOfLimit) // �������� or �p�x�I�[�o�[
            {

                for (int i = 0; i < joint.Length; i++)
                {
                    slider[i].GetComponent<Slider>().value = prevSliderVal[i];
                    angle[i] = prevAngle[i];
                }
                //���B�s�\�ł��錋�ʂ�\������
                ReachResult.GetComponent<TextMeshProUGUI>().text = "NG";
            }
            else      // ���{�b�g�̊֐ߊp�x���X�V
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
            point[0] = new Vector3(0f, 0f, 0f);
            wRotation[0] = Quaternion.AngleAxis(angle[0], axis[0]);
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
            // �ʒu�덷
            Vector3 perr = pos - point[6];
            // �p���덷
            Quaternion rerr = Quaternion.Euler(rot) * Quaternion.Inverse(wRotation[5]);
            // xyz����̉�]�ɕϊ�
            Vector3 rerrVal = new Vector3(rerr.eulerAngles.x, rerr.eulerAngles.y, rerr.eulerAngles.z);
            if (rerrVal.x > 180f) rerrVal.x -= 360f;
            if (rerrVal.y > 180f) rerrVal.y -= 360f;
            if (rerrVal.z > 180f) rerrVal.z -= 360f;
            var err = DenseMatrix.OfArray(new float[,]
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

        //InputField���̐��l��������
        public void ClickResetButton()
        {
            //WeldPointInput_pos_x = GameObject.Find("Input_X").GetComponent<TMP_InputField>();
            WeldPointInput_pos_x.GetComponent<TMP_InputField>().text = csvDatas[1][2].ToString();
            //WeldPointInput_pos_y = GameObject.Find("Input_Y").GetComponent<TMP_InputField>();
            WeldPointInput_pos_y.GetComponent<TMP_InputField>().text = csvDatas[1][3].ToString();
            //WeldPointInput_pos_z = GameObject.Find("Input_Z").GetComponent<TMP_InputField>();
            WeldPointInput_pos_z.GetComponent<TMP_InputField>().text = csvDatas[1][4].ToString();
            //WeldPointInput_ang_x = GameObject.Find("Input_RX").GetComponent<TMP_InputField>();
            WeldPointInput_ang_x.GetComponent<TMP_InputField>().text = csvDatas[1][5].ToString();
            //WeldPointInput_ang_y = GameObject.Find("Input_RY").GetComponent<TMP_InputField>();
            WeldPointInput_ang_y.GetComponent<TMP_InputField>().text = csvDatas[1][6].ToString();
            //WeldPointInput_ang_z = GameObject.Find("Input_RZ").GetComponent<TMP_InputField>();
            WeldPointInput_ang_z.GetComponent<TMP_InputField>().text = csvDatas[1][7].ToString();
        }

        public void ClickRegistButton()
        {
            Debug.Log("Button click!");
        }



        // Start is called before the first frame update
        void Start()
        {
            
            // csv�t�@�C��(DB)�̓ǂݍ���
            csvFile = Resources.Load("WeldPoint") as TextAsset; // Resouces����CSV�ǂݍ���
            StringReader reader = new StringReader(csvFile.text);
            // , �ŕ�������s���ǂݍ���
            // ���X�g�ɒǉ����Ă���
            while (reader.Peek() != -1) // reader.Peaek��-1�ɂȂ�܂�
            {
                string line = reader.ReadLine(); // ��s���ǂݍ���
                csvDatas.Add(line.Split(',')); // , ��؂�Ń��X�g�ɒǉ�
            }

            // csvDatas[�s][��]���w�肵�Ēl�����R�Ɏ��o����
            for (var x = 0; x < csvDatas.Count; x++)
            {
                for (var y = 0; y < csvDatas[x].Length; y++)
                {
                    Debug.Log(csvDatas[x][y]);
                }
            }

            //InputField��csv�̓��e����͂���
            WeldPointInput_pos_x = GameObject.Find("Input_X").GetComponent<TMP_InputField>();
            WeldPointInput_pos_x.GetComponent<TMP_InputField>().text = csvDatas[1][2].ToString();
            WeldPointInput_pos_y = GameObject.Find("Input_Y").GetComponent<TMP_InputField>();
            WeldPointInput_pos_y.GetComponent<TMP_InputField>().text = csvDatas[1][3].ToString();
            WeldPointInput_pos_z = GameObject.Find("Input_Z").GetComponent<TMP_InputField>();
            WeldPointInput_pos_z.GetComponent<TMP_InputField>().text = csvDatas[1][4].ToString();
            WeldPointInput_ang_x = GameObject.Find("Input_RX").GetComponent<TMP_InputField>();
            WeldPointInput_ang_x.GetComponent<TMP_InputField>().text = csvDatas[1][5].ToString();
            WeldPointInput_ang_y = GameObject.Find("Input_RY").GetComponent<TMP_InputField>();
            WeldPointInput_ang_y.GetComponent<TMP_InputField>().text = csvDatas[1][6].ToString();
            WeldPointInput_ang_z = GameObject.Find("Input_RZ").GetComponent<TMP_InputField>();
            WeldPointInput_ang_z.GetComponent<TMP_InputField>().text = csvDatas[1][7].ToString();


            //robot
            for (int i = 0; i < joint.Length; i++)
            {
                joint[i] = GameObject.Find("Joint_" + i.ToString());
            }

            //UI settings
            for (int i = 0; i < joint.Length; i++)
            {
                slider[i] = GameObject.Find("Slider_" + i.ToString());
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
                posText[i] = GameObject.Find("Ref_" + i.ToString());
                angText[i] = GameObject.Find("Ang_" + i.ToString());
            }

            // �C�j�V�����p���ł̊e�A�[���̐��@
            dim[0] = new Vector3(0f, 2f, 0f);
            dim[1] = new Vector3(4f, 0f, 0f);
            dim[2] = new Vector3(1f, 0f, 0f);
            dim[3] = new Vector3(2f, 0f, 0f);
            dim[4] = new Vector3(1f, 0f, 0f);
            dim[5] = new Vector3(2f, 0f, 0f);

            // �e��]���̕���
            axis[0] = new Vector3(0f, 1f, 0f);
            axis[1] = new Vector3(0f, 0f, 1f);
            axis[2] = new Vector3(0f, 0f, 1f);
            axis[3] = new Vector3(1f, 0f, 0f);
            axis[4] = new Vector3(0f, 0f, 1f);
            axis[5] = new Vector3(1f, 0f, 0f);

            // �C�j�V�����p���ł̉�]�p
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

            //����v�Z
            for (WeldPoint_ID = 0; WeldPoint_ID < 3; WeldPoint_ID++)
            {
                // WeldPoint�̃I�u�W�F�N�g������
                targetObject = GameObject.Find("WeldPoint_" + WeldPoint_ID);

                //���[�`���B�s�̌��ʗp�I�u�W�F�N�g�̌���
                ReachResult = GameObject.Find("P" + WeldPoint_ID + "_ReachResult");

                //WeldPoint�̍��W��inputField�ɃC���v�b�g
              //  P1_posText[0] = GameObject.Find("P1_Input_Px");

                //�I�u�W�F�N�g�̃��[���h���W���擾
                Vector3 worldPosition1 = targetObject.transform.position;
                Vector3 worldPosition2 = targetObject.transform.localEulerAngles;

                //�e�X���C�_�[�̒l���I�u�W�F�N�g�̍��W�ɕύX
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

                for (int i = 0; i < joint.Length; i++)
                {
                    sliderVal[i] = slider[i].GetComponent<Slider>().value;
                }

                //�X���C�_�[�̒l���I�u�W�F�N�g�̍��W�ɓ���
              //  targetObject.transform.position = new Vector3(sliderVal[0], sliderVal[1], sliderVal[2]);
                //targetObject.transform.localEulerAngles = new Vector3(sliderVal[3], sliderVal[4], sliderVal[5]);

                //�v�Z�p�֐��ɃX���C�_�[�̒l����
                pos.x = sliderVal[0];
                pos.y = sliderVal[1];
                pos.z = sliderVal[2];
                rot.x = sliderVal[3];
                rot.y = sliderVal[4];
                rot.z = sliderVal[5];

                // IK
                CalcIK();

                //���BNG�̏ꍇ�ɗ��p���钼�O�̊e��l��������
                for (int i = 0; i < joint.Length; i++)
                {
                    prevSliderVal[i]=0;
                    prevAngle[i] =0;
                }
                // �C�j�V�����p���ł̉�]�p
                angle[0] = 0f;
                angle[1] = 90f;
                angle[2] = -90f;
                angle[3] = 0f;
                angle[4] = 0f;
                angle[5] = 0f;
            }

        }
    

        // Update is called once per frame
        void Update()
        {
            
             
            WeldPoint_ID = 0;
            // WeldPoint�̃I�u�W�F�N�g������
            targetObject = GameObject.Find("WeldPoint_" + WeldPoint_ID);
            //���[�`���B�s�̌��ʗp�I�u�W�F�N�g�̌���
            ReachResult = GameObject.Find("P" + WeldPoint_ID + "_ReachResult");

            //�I�u�W�F�N�g�̃��[���h���W���擾
            Vector3 worldPosition1 = targetObject.transform.position;
            Vector3 worldPosition2 = targetObject.transform.localEulerAngles;

            // Updata�̍ŏ��̈�񂾂��I�u�W�F�N�g�̈ʒu
            if (isCalledOnce==false)
            {
            //�e�X���C�_�[�̒l���I�u�W�F�N�g�̍��W�ɕύX
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
            }

            for (int i = 0; i < joint.Length; i++)
                {
                    sliderVal[i] = slider[i].GetComponent<Slider>().value;
                }     

                //�X���C�_�[�̒l���I�u�W�F�N�g�̍��W�ɓ���
                targetObject.transform.position = new Vector3(sliderVal[0], sliderVal[1], sliderVal[2]);
                targetObject.transform.localEulerAngles = new Vector3(sliderVal[3], sliderVal[4], sliderVal[5]);

                //�v�Z�p�֐��ɃX���C�_�[�̒l����
                pos.x = sliderVal[0];
                pos.y = sliderVal[1];
                pos.z = sliderVal[2];
                rot.x = sliderVal[3];
                rot.y = sliderVal[4];
                rot.z = sliderVal[5];

                // IK
                CalcIK();
            isCalledOnce = true;


        }
    }
}
