# RaspberryPi程式安裝步驟
1. **ROS2 package**用來搭配MATLAB GUI程式
2. **本地測試程式**可以在沒有MATLAB的狀況下測試robot是否運作正常

## ROS2 package
note:請先確認ROS2 Humble Hawksbill安裝完成且正確
1. 在樹梅派的家目錄建立一個ROS2 workspace  
```bash
mkdir -p ~/ros2_ws/src
```
2. 將mecanum_wmr資料夾全部複製到~/ros2_ws/src目錄
3. 回到~/ros2_ws目錄編譯程式
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash #開機後輸入一次就好
colcon build --symlink-install --packages-select mecanum_wmr
```
沒有出現錯誤就安裝完成了  

## 本地測試程式
1. 在家目錄建立/MecanumWMR資料夾，然後把MecanumWMR裡的全部檔案複製進去
```bash
mkdir MecanumWMR
```
2. 建立/build資料夾
```bash
cd ~/MecanumWMR
mkdir build
```
3. 使用cmake
```bash
cd ~/MecanumWMR/build
cmake ../
```
4. 編譯程式
```bash
make
```
沒有出現錯誤就安裝完成了  