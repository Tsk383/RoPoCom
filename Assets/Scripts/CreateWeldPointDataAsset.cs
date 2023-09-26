using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal.Profiling.Memory.Experimental.FileFormat;
using UnityEngine.UIElements;

// AssetDatabaseを使用しているため、ビルド時には含めないようにしないとビルドエラーが起きる
#if UNITY_EDITOR
public class CreateWeldPointDataAssetFromCsv
{
    private const string AssetPath = "Assets/Resources/";
    private const string CsvPath = "Assets/Resources/WeldPoint.csv";
    private float values;

    // MenuItem属性を付けることでEditorの上部メニューに`ScriptableObjects > CreateEnemyParamAsset`が表示されます
    // 押下すると`CreateEnemyParamDataAsset()`が実行されます
 //   [MenuItem("ScriptableObjects/CreateWeldPointDataAsset")]
    private static void CreateWeldPointDataAsset()
    {
        //  var weldPointDataAsset = CreateInstance<WeldPointDataAsset>();

        // この辺で外部ファイルパスを用いてデータを読みこみ、
        // 作成したenemyParamAssetに値を流し込む処理を挟む....
        // 例えばenemyParamAsset.EnemyParamList.Add(hogeParam); 的な

        // 流し込んだ後は実際に作成します
        // ここで作ったアセットの置き場所であるパスの指定もできます
        // var assetName = $"{AssetPath}{WeldPoint}Data.asset";
        //AssetDatabase.CreateAsset(weldPointDataAsset, assetName);

        // Asset作成後、反映させるために必要なメソッド
    //    AssetDatabase.Refresh();
    }
    void Start()
    {
        // csvファイルを開く
        using (StreamReader reader = new StreamReader(CsvPath))
        {
            // csvファイルの行を読み込む
            while (!reader.EndOfStream)
            {
                // 行を分割して配列に格納
                string[] values = reader.ReadLine().Split(',');

                // 座標を配列に追加
                //positions.Add(new Vector3(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2])));
            }
        }
        Debug.Log(values);
    }
}
#endif