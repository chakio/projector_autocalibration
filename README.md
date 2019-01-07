[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
# projector_autocalibration
![result](https://github.com/chakio/projector_autocalibration/blob/master/media/auto_calibration.gif)  
プロジェクターで投影する画像を投影変面に対して自動補正します。

## Description
今後の情報提示方法や表現方法の拡張を見据えると、プロジェクターが自由に動きまわることが予期されます。  
可動式プロジェクターを実現する上での必要事柄として、投影される画像の歪みを動的に補正することが挙げられます。   
動的なキャリブレーション手法は多く検討されていますが、マーカを用いたものが多いです。  
そこで、今回はマーカレスの動的キャリブレーションを検討してみました。  

## Hardware requirement 
* Projector : ASUS P3B
* 3D camera : ASUS Xtion PRO LIVE
* PC : Alienware 13  

<img src="https://github.com/chakio/projector_autocalibration/blob/master/media/hardware_component.jpg" width="500px"> 

## Software requirement 
* OS : Ubuntu 16.04
* ROS : kinetic  
* openFrameworks : v0.10.0 

* visualstudio code

<img src="https://github.com/chakio/projector_autocalibration/blob/master/media/software_component.png" width="1000px"> 


## Marker less calibration
3Dカメラを用いた平面検出をした後に、カメラと平面の相対位置と投影したい画像サイズに基づき  
その平面に対してどこに投影するかを決定しています。
そして、その位置に投影できるように画像を[アフィン変換](https://ja.wikipedia.org/wiki/%E3%82%A2%E3%83%95%E3%82%A3%E3%83%B3%E5%86%99%E5%83%8F)した後に投影します。  
<img src="https://github.com/chakio/projector_autocalibration/blob/master/media/plane_detection.gif" width="500px">   

openFrameworksを[ROS化](https://github.com/chakio/ofxRosTemplate)し、点群取得と処理はROS、画像変形と投影はopenFrameworksで作成しています。  
* 点群取得 : OpenNI
* 点群処理 : PCL (Point Cloud Library)
* 画像処理 : ofxOpenCV

## Setup
### OpenNI2
* ```$ sudo apt-get install ros-kinetic-rgbd-launch ros-kinetic-openni2-camera ros-kinetic-openni2-launch```  

### ROS×openFrameworks
* [openFrameworksのROS対応を施したテンプレートについて](https://github.com/chakio/ofxRosTemplate)


### Usage
* ```roslaunch openni2_launch openni2.launch```
* ```roslaunch projector_autocalibration_launch auto_calibration.launch```   

カレントディレクトリを  
```hogehoge/projector_autocalibration/ofxTransformImage```  
に移動した後に  
* ```./bin/ofTransformImage```

## Lisence
[MIT](https://github.com/chakio/projector_autocalibration/blob/master/LICENSE)

## Author
[chakio](https://github.com/chakio)