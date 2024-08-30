# Mecanum Wheeled Mobile Robot
這是一個軟硬體皆自行設計整合的四Mecanum輪型移動機器人專案。主要功能為筆電利用MATLAB GUI程式規劃出一序列的點到點避障路徑，經由ROS2對機器人下指令移動，並在移動中即時回傳機器人目前位置顯示在GUI上。機器人程式採用C++與C程式語言在ROS2環境下運作。

## 主要硬體
1. **計算單元** : RaspberryPi model 4B+、STM32 Nucleo L476RG
2. **馬達** : TT馬達 x 4
3. **馬達驅動器** : TB6612FNG x 2
4. **IMU** : MPU9250
5. **電源供應** : 小米行動電源5000mAh、18650鋰電池 x 3、LM2596

## 程式開發工具與環境
STM32CubeIDE 1.16.0  
RaspberryPi : Ubuntu server 22.04.03 LTS (64bit)、ROS2 Humble Hawksbill

## 安裝與使用
### 安裝步驟
1. STM32
請看STM32資料夾的README檔案
2. RaspberryPi
請看RaspberryPi資料夾的README檔案
### 使用教學

#### MecanumWMR API

## 成果展示


## 致謝
此專案引用並修改了以下專案的部分程式碼  
* Hideaki Tai (hideakitai)'s GitHub project: https://github.com/hideakitai/MPU9250  
* Alex Baucom (alexbaucom17)'s GitHub project: https://github.com/alexbaucom17/DominoRobot/blob/master/src/robot/src/SmoothTrajectoryGenerator.cpp#L234

## 聯絡我們
姚昱志  
顏佑丞 polie4523@gmail.com  
蕭丞佑  