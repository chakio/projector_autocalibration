[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
# projector_autocalibration
![result](https://github.com/chakio/projector_autocalibration/blob/master/media/auto_calibration.gif)  
プロジェクターで投影する画像を自動補正します。

### Description
今後の情報提示方法や表現方法の拡張を見据えると、プロジェクターが自由に動きまわることが予期されます。  
可動式プロジェクターを実現する上で問題となる事柄の一つとして、投影される画像の歪みを動的に補正することが挙げられます。   
動的なキャリブレーション手法は多く検討されていますが、マーカを用いたものが多いです。  
そこで、今回はマーカレスの動的キャリブレーションを検討してみました。  

### Hardware composition
* Projector : ASUS P3B
* 3D camera : ASUS Xtion PRO LIVE
<img src="https://github.com/chakio/projector_autocalibration/blob/master/media/hardware_component.jpg" width="500px"> 

### Software composition
* OS : Ubuntu 16.04
* ROS : kinetic  
* openFrameworks :   v0.10.0
* PC : Alienware 13 

* visualstudio code

<img src="https://github.com/chakio/projector_autocalibration/blob/master/media/software_component.png" width="1000px"> 

マーカを使用する代わりに、  
3Dカメラを用いた平面検出をした後に、カメラと平面の相対位置と投影したい画像サイズに基づき  
その平面に対してどこに投影するかを決定しています。
そして、その位置に投影できるように画像をアフィン変換した後に投影します。  

openFrameworksをROS化し、点群取得と処理はROS、画像変形と投影はopenFrameworksで作成しています。  
* 点群取得 : OpenNI
* 点群処理 : PCL
* 画像処理 : ofxOpenCV

### Setup
* OpenPoseを[インストール方法](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)に従いインストールした後に、  [Python API](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#python-api)をインストール

* （C++ APIなどの選択肢もあるが、CV::BridgeなどがOpencvのバージョンなどの問題でうごかなかった）

* （openpose_wrapper.py というサンプルコードを改変し、ros化したが、スレッドが別れないようにする工夫が必要です。（あやふや））

### Useage
* ```rosrun openpose_ros openpose_wrapper.py```
