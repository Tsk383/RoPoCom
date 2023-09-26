using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal.Profiling.Memory.Experimental.FileFormat;
using UnityEngine.UIElements;

// AssetDatabase���g�p���Ă��邽�߁A�r���h���ɂ͊܂߂Ȃ��悤�ɂ��Ȃ��ƃr���h�G���[���N����
#if UNITY_EDITOR
public class CreateWeldPointDataAssetFromCsv
{
    private const string AssetPath = "Assets/Resources/";
    private const string CsvPath = "Assets/Resources/WeldPoint.csv";
    private float values;

    // MenuItem������t���邱�Ƃ�Editor�̏㕔���j���[��`ScriptableObjects > CreateEnemyParamAsset`���\������܂�
    // ���������`CreateEnemyParamDataAsset()`�����s����܂�
 //   [MenuItem("ScriptableObjects/CreateWeldPointDataAsset")]
    private static void CreateWeldPointDataAsset()
    {
        //  var weldPointDataAsset = CreateInstance<WeldPointDataAsset>();

        // ���̕ӂŊO���t�@�C���p�X��p���ăf�[�^��ǂ݂��݁A
        // �쐬����enemyParamAsset�ɒl�𗬂����ޏ���������....
        // �Ⴆ��enemyParamAsset.EnemyParamList.Add(hogeParam); �I��

        // �������񂾌�͎��ۂɍ쐬���܂�
        // �����ō�����A�Z�b�g�̒u���ꏊ�ł���p�X�̎w����ł��܂�
        // var assetName = $"{AssetPath}{WeldPoint}Data.asset";
        //AssetDatabase.CreateAsset(weldPointDataAsset, assetName);

        // Asset�쐬��A���f�����邽�߂ɕK�v�ȃ��\�b�h
    //    AssetDatabase.Refresh();
    }
    void Start()
    {
        // csv�t�@�C�����J��
        using (StreamReader reader = new StreamReader(CsvPath))
        {
            // csv�t�@�C���̍s��ǂݍ���
            while (!reader.EndOfStream)
            {
                // �s�𕪊����Ĕz��Ɋi�[
                string[] values = reader.ReadLine().Split(',');

                // ���W��z��ɒǉ�
                //positions.Add(new Vector3(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2])));
            }
        }
        Debug.Log(values);
    }
}
#endif