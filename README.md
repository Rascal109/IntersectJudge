# IntersectJudge
## 注意
Mayaのプラグイン開発の練習として作ってみたものである。
## 用途と原理
<p align = "center">
  <img src = "https://github.com/user-attachments/assets/78f93fde-5b5c-4306-80b1-0eeee0e67e49"　alt="Image 1" height="300"> 
  <img src = "https://github.com/user-attachments/assets/a9fcc20a-ed3a-4476-b10f-e632575c0c8c"　alt="Image 2" height="300"/>
</p>
   
* Maya上において, ある一つのオブジェクトに対して面の範囲を選択した際に、その範囲内で物体が交差しているかどうかを判定するプラグイン。
交差している場合は、その面を赤く表示するようにしている。
* 接触判定にはBVHを用いた空間分割法を自前で実装している。そこから交差する面を探索し、交差する可能性が高い面では、厳密な面同士の交差判定をおこなっている。
* BVHの構築では、階層の深さに応じて分割する軸方向を決めるという簡易的な方法をとっている。
## 使用方法
1.  `x64/Debug`フォルダの中にある`intersectJudge.mll`ファイルをMayaの`plug-insフォルダ`に入れる。
2.  Mayaのアプリ上で、ウィンドウ->設定/プリファレンス->プラグインマネージャからプラグインの選択(ON/OFF)が可能。
3.  プラグインをONにした後、スクリプトエディタに`Intersect` と入力すると、コールバック関数が有効になる。
4.  マウスでオブジェクトの面の範囲選択をするたびに交差している箇所が赤くなる。

* からフォルダの中にある`intersectJudge.mll`ファイルをMayaの`plug-insフォルダ`に入れる。
2.  Mayaのアプリ上で、ウィンドウ->設定/プリファレンス->プラグインマネージャからプラグインの選択(ON/OFF)が可能。
3.  プラグインをONにした後、スクリプトエディタに`Intersect` と入力すると、コールバック関数が有効になる。
4.  マウスでオブジェクトの面の範囲選択をするたびに交差している箇所が赤くなる。

* カラーが反映されない場合は、「カラーセット」を表示し、「適用」をクリックする必要がある。
* 解除するときは、**注目しているオブジェクトを選択した状態で**OFFにする。(そうしないと色が残ってしまう)
* 色が残った場合は、再度プラグインをONにして、適当にオブジェクトの面を選択すれば消える。
## 今後の展望
- そのオブジェクト内でポリゴンを動かすたびに、面同士の交差判定をおこなえるようにしたい。
- シーン内の複数オブジェクトに対しても、ポリゴンの交差判定をおこなえるようにしたい。
