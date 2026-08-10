# roarm_m2

## Python API 使用说明

```python
from roarm_sdk.roarm import roarm

# 串口通信示例
roarm = roarm(roarm_type="roarm_m2", port="/dev/ttyUSB0", baudrate=115200)

# Http通信示例（仅发控制指令；读反馈 / 拖动示教请用串口）
# 注意：http通信需要先连接在同一个wifi下，host为机械臂的ip地址
#roarm = roarm(roarm_type="roarm_m2", host="192.168.4.1")

# 可选：夹爪类型（默认 "angular_direct"，与旧行为兼容）
# RoArm-M2-GA 请使用 gripper_type="angular_gear"
#roarm = roarm(roarm_type="roarm_m2", port="/dev/ttyUSB0", baudrate=115200, gripper_type="angular_gear")

# 可选：调试时打印收发数据（默认 False）
#roarm = roarm(roarm_type="roarm_m2", port="/dev/ttyUSB0", baudrate=115200, debug=True)

# 获取所有关节当前的角度
value = roarm.joints_angle_get()
print(value)

# 将 关节1 移动到 90度，速度设置为 1000步/秒，加速度设置为 50（单位为 100 步/秒^2）
roarm.joint_angle_ctrl(joint=1,angle=90,speed=1000,acc=50)
```

### 构造参数

- `roarm_type`: `"roarm_m2"` 或 `"roarm_m3"`
- `port` / `baudrate`: 串口通信参数；或使用 `host` 进行 HTTP 通信（HTTP 仅发控制，反馈需串口）
- `gripper_type`: 夹爪类型，默认 `"angular_direct"`
  - `"angular_direct"`: 夹爪角度/弧度会做取反处理（旧行为；RoArm-M2-S / M2-Pro）
  - `"angular_gear"`: 夹爪值直传，不做取反（RoArm-M2-GA）；关节弧度仍可用 `0` 关闭 / `1.57` 开启
- `debug`: 是否打印收发数据，默认 `False`；设为 `True` 时打印

### 1. 整体运行状态

#### `echo_set(cmd)`

- **功能:** 设置回声模式

- **参数:**
  - `cmd:` [0, 1], type : int
    - 0: 关闭回声模式
    - 1: 开启回声模式

#### `middle_set()`

- **功能:** 将当前位置校准为中位

#### `move_init()`

- **功能:** 移动到初始位置

### `led_ctrl(led)`
- **功能:** 设置LED灯的状态

- **参数:**
  - `led:` [0, 255], type : int
    - 0: 最暗
    - 255: 最亮

#### `torque_set(cmd)`
- **功能:** 设置所有关节的力矩是否开启

- **参数:**
  - `cmd:` [0, 1], type : int
    - 0: 关闭所有关节的力矩
    - 1: 开启所有关节的力矩

#### `dynamic_adaptation_set(mode, torques)`
- **功能:** 设置所有关节的力矩自适应模式

- **参数:**
  - `mode:` [0, 1], type : int
    - 0: 关闭力矩自适应模式
    - 1: 开启力矩自适应模式
  - `torques:` [0, 1000], type : list[int]，unit : 0.1% 堵转扭矩（1000 = 100%）
    - `[关节1力矩阈值, 关节2力矩阈值, 关节3力矩阈值, 关节4力矩阈值]`
    - 超过该阈值时，关节随外力转动

####  `feedback_get()`
- **功能:** 获取末端坐标与各关节弧度反馈（需串口；HTTP 模式不支持读反馈）

- **返回值:** list[float]
  - `[x, y, z, 关节1弧度, 关节2弧度, 关节3弧度, 关节4弧度]`
  - 坐标 `x/y/z`：单位 毫米
  - 关节弧度：单位 弧度

### 2. 关节控制

#### `joint_radian_ctrl(joint, radian, speed, acc)`
- **功能:** 将指定关节移动到指定弧度

- **参数:**
  - `joint:` [1, 4], type : int
  - `radian:` type : float，unit : 弧度
    - 1: 关节1，[-3.1415926, 3.1415926]
    - 2: 关节2，[-1.5707963, 1.5707963]
    - 3: 关节3，[0, 3.1415926]
    - 4: 关节4，[0, 1.5707963]
  - `speed:` [1, 4096], type : int，unit : 步/秒
  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `joints_radian_ctrl(radians, speed, acc)`
- **功能:** 将所有关节移动到指定弧度

- **参数:**
  - `radians:` type : list[float]
    - `[关节1弧度, 关节2弧度, 关节3弧度, 关节4弧度]`
    - 1: 关节1，[-3.1415926, 3.1415926]
    - 2: 关节2，[-1.5707963, 1.5707963]
    - 3: 关节3，[0, 3.1415926]
    - 4: 关节4，[0, 1.5707963]
  - `speed:` [1, 4096], type : int，unit : 步/秒
  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `joints_radian_get()`
- **功能:** 获取所有关节的弧度

- **返回值:** list[float], unit : 弧度
  - [关节1弧度, 关节2弧度, 关节3弧度, 关节4弧度]

#### `joint_angle_ctrl(joint, angle, speed, acc)`
- **功能:** 将指定关节移动到指定角度

- **参数:**
  - `joint:` [1, 4], type : int
  - `angle:` type : float，unit : 度
    - 1: 关节1，[-180, 180]
    - 2: 关节2，[-90, 90]
    - 3: 关节3，[0, 180]
    - 4: 关节4，[0, 90]

  - `speed:` [1, 4096], type : int，unit : 步/秒
  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `joints_angle_ctrl(angles, speed, acc)`
- **功能:** 将所有关节移动到指定角度

- **参数:**
  - `angles:` type : list[float]
    - `[关节1角度, 关节2角度, 关节3角度, 关节4角度]`
    - 1: 关节1，[-180, 180]
    - 2: 关节2，[-90, 90]
    - 3: 关节3，[0, 180]
    - 4: 关节4，[0, 90]
  - `speed:` [1, 4096], type : int，unit : 步/秒
  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `joints_angle_get()`
- **功能:** 获取所有关节的角度

- **返回值:** list[float], unit : 度
  - [关节1角度, 关节2角度, 关节3角度, 关节4角度]

#### `drag_teach_start(filename)`
- **功能:** 开始拖拽教学（需串口；HTTP 模式不支持录制）

- **参数:**
  - `filename:` filename, type : str

#### `drag_teach_replay(filename)`
- **功能:** 播放拖拽教学（可串口或 HTTP 发控制）

- **参数:**
  - `filename:` filename, type : str

### 3. 夹爪控制

#### `gripper_mode_set(mode)`
- **功能:** 设置夹爪模式

- **参数:**
  - `mode:` [0, 1], type : int
    - 0: 夹爪模式
    - 1: 手腕模式

#### `gripper_radian_ctrl(radian, speed, acc)`
- **功能:** 将夹爪移动到指定弧度

- **参数:**
  - `radian:` [0, 1.5707963], type : float，unit : 弧度

  - `speed:` [1, 4096], type : int，unit : 步/秒

  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `gripper_angle_ctrl(angle, speed, acc)`
- **功能:** 将夹爪移动到指定角度

- **参数:**
  - `angle:` [0, 90], type : float，unit : 度

  - `speed:` [1, 4096], type : int，unit : 步/秒

  - `acc:` [1, 254], type : int，unit : 100 步/秒^2

#### `gripper_radian_get()`
- **功能:** 获取夹爪的弧度

- **返回值:** float，unit : 弧度

#### `gripper_angle_get()`
- **功能:** 获取夹爪的角度

- **返回值:** float，unit : 度

### 4. 位置控制

#### `pose_ctrl(pose)`
- **功能:** 移动到指定位置

- **参数:**
  - `pose:` type : list[float]
    - `[坐标x, 坐标y, 坐标z, 夹爪角度]`
    - 坐标单位：毫米，角度单位：度
    - x: [-500, 500]
    - y: [-500, 500]
    - z: [-600, 600]
    - 夹爪角度: [0, 90]

#### `pose_get()`
- **功能:** 获取当前位置

- **返回值:** list[float]
  - [坐标x, 坐标y, 坐标z, 夹爪角度]
  - 坐标unit : 毫米，角度unit : 度

### 5. WiFi控制

#### `wifi_on_boot(wifi_cmd)`
- **功能:** 设置启动时WiFi模式

- **参数:**
  - `wifi_cmd:` [0, 3], type : int
    - 0: 关闭WiFi
    - 1: AP模式
    - 2: STA模式
    - 3: AP+STA模式

#### `ap_set(ssid, password)`
- **功能:** 设置AP模式下的WiFi名称和密码

- **参数:**
  - `ssid:` type : str

  - `password:` type : str

#### `sta_set(ssid, password)`
- **功能:** 设置STA模式下的WiFi名称和密码
- **参数:**
  - `ssid:` type : str

  - `password:` type : str

#### `apsta_set(ap_ssid, ap_password, sta_ssid, sta_password)`
- **功能:** 设置AP+STA模式下的WiFi名称和密码

- **参数:**
  - `ap_ssid:` type : str

  - `ap_password:` type : str

  - `sta_ssid:` type : str

  - `sta_password:` type : str

#### `wifi_config_creat_by_status()`
- **功能:** 根据当前WiFi状态生成WiFi配置文件

#### `wifi_config_creat_by_input(ap_ssid, ap_password, sta_ssid, sta_password)`
- **功能:** 根据输入生成WiFi配置文件

- **参数:**
  - `ap_ssid:` type : str

  - `ap_password:` type : str

  - `sta_ssid:` type : str

  - `sta_password:` type : str

#### `wifi_stop()`
- **功能:** 关闭WiFi