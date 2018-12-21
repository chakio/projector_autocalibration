[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
# OpenPose-ROS
![result](https://github.com/chakio/openpose_ros/blob/master/media/openpose3D.gif)  
[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)のラッパーです。

### Description
ROSにてOpenPoseを用いた関節位置推定を行うためのプログラムです。  
RGB-Dカメラを用いることで、関節位置の3次元座標を取得しています。  
他にもいくつか機能がついていますので参考にしてみてください。

### Feature
* 想定使用法:ロボットにノートPCを搭載したうえでの、人の3次元関節位置推定
* OpenPoseの3次元化：PCDの画素マッチングによる３次元座標取得
* TFの管理：OpenPoseの処理時間の考慮
* 出力：今回はsensor_msgs::Pointcloud2を使用(新規メッセージの作成は複数PC上でのSubscribeの際手間)。オリジナルのFieldを定義可能なためconfidenceなどもやりとり可能

### Environment
* OpenPose : v1.3.0
* ROS : kinetic  

* Robot : Toyota HSR
* PC : Alienware 13

### Setup
* OpenPoseを[インストール方法](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)に従いインストールした後に、  [Python API](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#python-api)をインストール

* （C++ APIなどの選択肢もあるが、CV::BridgeなどがOpencvのバージョンなどの問題でうごかなかった）

* （openpose_wrapper.py というサンプルコードを改変し、ros化したが、スレッドが別れないようにする工夫が必要です。（あやふや））

### Useage
* ```rosrun openpose_ros openpose_wrapper.py```
