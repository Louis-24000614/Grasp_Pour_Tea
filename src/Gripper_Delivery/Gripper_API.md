没问题！既然我们要追求“保姆级”的交接体验，这份文档需要包含**环境配置、权限处理、启动流程和 API 调用**的所有细节。

以下是为你优化后的完整版 **`Gripper_API.md`**：

---

# 🤖 OmniPicker 夹爪控制 API 手册 (V2.0)

本手册旨在帮助视觉算法工程师或大模型应用开发者，通过简单的 Python API 快速控制 **OmniPicker 自适应夹爪**。底层基于 ROS 2 Service 通信，具备**动作阻塞等待**和**堵转感知（抓取确认）**功能。

---

## 🛠️ 1. 环境准备 (准备工作)

在运行代码前，请确保已完成以下配置：

### 1.1 安装 Python 依赖
在交接包根目录下运行：
```bash
pip install -r requirements.txt
```
*(主要安装 `pyserial` 库用于串口通信)*

### 1.2 编译 ROS 2 接口
必须编译自定义接口包，否则程序无法识别通信格式：
1. 将 `omnipicker_interfaces` 文件夹放入你的 ROS 2 工作空间的 `src` 目录。
2. 在工作空间根目录运行：
   ```bash
   colcon build --packages-select omnipicker_interfaces
   source install/setup.bash
   ```

### 1.3 串口权限设置 (重要)
如果提示 `/dev/ttyUSB0` 权限不足，请执行以下命令：
```bash
sudo chmod 666 /dev/ttyUSB0
```

---

## 🚀 2. 启动流程 (运行顺序)

请严格遵守以下启动顺序：

1. **第一步：启动底层驱动**（负责与硬件通信）
   打开一个终端，运行：
   ```bash
   python3 rs485_driver.py
   ```
   *看到 `✅ RS485 串口已就绪` 后即可继续。*

2. **第二步：调用上层 API**
   在你的业务代码（如视觉抓取程序）中直接 `import` 即可。

---

## 📖 3. API 使用说明

### 3.1 初始化

```python
from gripper_api import GripperCommander
import rclpy

rclpy.init()
gripper = GripperCommander() 
```

### 3.2 核心方法

#### 🫱 `gripper.open(speed=0.8)`
将夹爪完全张开，用于松开物体。
* **参数:** `speed` (float, 0.0~1.0): 运动速度。
* **返回值:** `(success, code)`
    * `success`: bool, 是否成功。
    * `code`: str, 状态码，通常为 `'success'`。

#### ✊ `gripper.close(force=0.5, speed=0.8)`
将夹爪完全闭合。**具备堵转感知**，抓到物体会立即停止。
* **参数:** * `force` (float, 0.0~1.0): 夹持力上限。
    * `speed` (float, 0.0~1.0): 闭合速度。
* **返回值:** `(success, code)`
    * `code == 'grasped'`: **成功抓到物体**。
    * `code == 'success'`: 夹爪完全闭合（抓空了）。

#### 🤏 `gripper.custom_grasp(pos, force, speed=0.5)`
移动到特定位置（0.0=闭合，1.0=完全打开）。
* **参数:** `pos` (float): 目标位置百分比。

---

## 📊 4. 状态码 (Result Code) 说明

| 状态码 | 含义 | 后续建议 |
| :--- | :--- | :--- |
| `success` | 动作正常完成 | 夹爪已到达指定位置 |
| `grasped` | **已抓到物体** | 触发堵转保护，可以进行下一步搬运 |
| `dropped` | 物体掉落 | 抓取过程中传感器检测到位移异常 |
| `timeout` | 5秒内无响应 | 检查硬件连接或串口是否松动 |
| `error` | 通信或代码异常 | 检查驱动节点是否崩溃 |

---

## 💡 5. 快速实战示例

```python
import rclpy
from gripper_api import GripperCommander

def main():
    rclpy.init()
    gripper = GripperCommander()

    # 1. 准备：张开夹爪
    gripper.open()

    # 2. 视觉引导机械臂运动... (此处省略)

    # 3. 执行抓取
    success, code = gripper.close(force=0.6, speed=0.5)

    if code == 'grasped':
        print("🎉 抓取成功！正在搬运...")
    elif code == 'success':
        print("❌ 抓空了，准备重试。")
    
    # 4. 释放
    gripper.open()

if __name__ == '__main__':
    main()
```

---

### 📬 开发者说明
如有接口变动需求，请联系：`[你的名字]`
代码版本：`V2.0` (基于 ROS 2 Foxy/Humble)