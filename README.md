# Mecanum Wheeled Mobile Robot
這是一個軟硬體皆自行設計整合的四Mecanum輪型移動機器人專案。主要功能為筆電利用MATLAB GUI程式規劃出一序列的點到點避障路徑，經由ROS2對機器人下指令移動，並在移動中即時回傳機器人目前位置顯示在GUI上。機器人程式採用C++與C程式語言在ROS2環境下運作。

## 主要硬體
1. **計算單元** : RaspberryPi model 4B+、STM32 Nucleo L476RG
2. **馬達** : TT馬達 x 4
3. **馬達驅動器** : TB6612FNG x 2
4. **IMU** : MPU9250
5. **電源供應** : 小米行動電源20000mAh、18650鋰電池 x 3、LM2596

## 程式開發工具與環境
STM32CubeIDE 1.16.0  
RaspberryPi : Ubuntu server 22.04.03 LTS (64bit)、ROS2 Humble Hawksbill

## 安裝與使用

### 安裝步驟
1. **STM32**  
請看STM32資料夾的README檔案
2. **RaspberryPi**  
請看RaspberryPi資料夾的README檔案


### 使用教學
* 樹梅派一開機就在家目錄先開啟pigpio daemon process，開機後只須執行一次  
```bash
sudo pigpiod
```
* 不再使用後就關閉pigpio daemon process  
```bash
sudo killall pigpiod
```
* 如果程式不正常停止導致馬達持續轉動就按下STM32板上面的黑色重置按鈕  
#### ROS2
程式安裝好後，打開新終端機，在ROS2的workspace目錄使用啟動文件執行節點  
```bash
source /opt/ros/humble/setup.bash #只需在第一次輸入
source install/local_setup.bash #只需在第一次輸入
ros2 launch mecanum_wmr control_node_launch.py
```
成功後console會出現"Waiting for topic..."字串。要讓節點停止請按ctrl + c  

#### 本地測試(不使用ROS2)
在MecanumWMR專案資料夾中的/build目錄執行主程式。功能為輸入三次目標點讓robot移動3次。  
```bash
./main
```
測試程式功能寫在main.cpp檔案，使用者可自行修改並在/build輸入make重新編譯安裝。  

#### MecanumWMR class API
```C++
bool init(void)
void shutdown(void)
void setInitCond(const float x_0, const float y_0, const float theta_0)
void setPIDgain_translation(const float kp, const float ki, const float kb)
void setPIDgain_rotation(const float kp, const float ki, const float kb)
void setVelocityLimit(const float linear_v_max, const float angular_v_max)
void Log(bool islog)
std::array<float, 3> getPosition(void) const
bool Point2PointMove(const std::array<float, 3> target_point)
```


## 成果展示


## 致謝
此專案引用並修改了以下專案的部分程式碼:  
* [Hideaki Tai (hideakitai)'s GitHub project](https://github.com/hideakitai/MPU9250)  
* [Alex Baucom (alexbaucom17)'s GitHub project](https://github.com/alexbaucom17/DominoRobot/blob/master/src/robot/src/SmoothTrajectoryGenerator.cpp#L234)

## 聯絡我們
姚昱志 Gary Yao :  
顏佑丞 Eric Yan : polie4523@gmail.com  
蕭丞佑 Bryan Hsiao : bryanhsiao200200000510@gmail.com