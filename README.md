# ros2_temperature_tracker
CPUとGPUの温度を監視・配信するためのROS 2パッケージ

# 環境
- Ubuntu22.04
- ROS 2 Humble 

# gputilのインストール
```bash
sudo pip3 install gputil
```

# インストール
```bash
cd ~/ros2_ws/src
git clone git@github.com:robohase/ros2_cpu_monitor.git
cd ~/ros2_ws
rosdep install -r -y -i --from-paths .
```

# ビルド
```bash
cd ~/ros2_ws/src
colcon build --symlink-install
source install/setup.bash
```

# 実行
## ターミナル1
```bash
ros2 launch temperature_tracker temperature_tracker.launch.py
```

## ターミナル2 
```bash
ros2 topic echo /cpu_temperature
```

## ターミナル3
```bash
ros2 topic echo /gpu_temperature
```


## パラメータ
### publish_gpu_temperature
GPU温度の配信を有効にします。

>注意: NvidiaのGPUを使用している場合のみ有効にしてください。他のGPUはサポートされていません。
>>NvidiaのGPUを搭載しているにもかかわらずノードで検出されない場合は、ターミナルで'nvidia-smi'コマンドを試してください。

### publish_cpu_temperature
CPU温度の配信を有効にします。

>注意: これはシステム内のサーマルファイルから直接読み取ることで動作します。Ubuntu 22.04でのみテストされています。

### cpu_type_id
/sys/class/thermal/thermal_zone*/typeで見つかる文字列値です。

>注意: IntelのCPUであれば"x86_pkg_temp"です。CPUアーキテクチャによって異なる場合があります。

### gpu_output_topic
GPU温度を配信する出力トピック名を表す文字列値です。

### cpu_output_topic
CPU温度を配信する出力トピック名を表す文字列値です。

### publish_rate [s]
温度を配信する間隔を秒単位で指定します。