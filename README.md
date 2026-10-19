# TurtleBot 4 - Bài thực hành 05

Package ROS 2 `tb4_assignment5` cho **Ubuntu 22.04 + ROS 2 Humble**, sử dụng TurtleBot 4 Simulator, Nav2 và YOLO.

## 1. Yêu cầu

- Ubuntu 22.04 (có thể sử dụng WSL2).
- ROS 2 Humble đã cài đặt.
- Kết nối Internet khi cài thư viện và tải model YOLO lần đầu.

## 2. Cài đặt dependencies

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-pip python3-yaml python3-opencv \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-cv-bridge \
  ros-humble-irobot-create-msgs
```

## 3. Thêm package vào workspace

Đặt thư mục `tb4_assignment5` (chứa `package.xml`, `setup.py`, `launch/`, `config/`...) vào `~/ros2_ws/src/`.

Nếu tải từ GitHub:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <URL_REPO_BAI_5> tb4_assignment5
```

> Thay `<URL_REPO_BAI_5>` bằng đường dẫn repository thực tế. Nếu đã có source code trong `src/`, bỏ qua bước clone.

Cài YOLO và các thư viện Python từ thư mục package:

```bash
cd ~/ros2_ws/src/tb4_assignment5
python3 -m pip install --user -r requirements-yolo.txt
```

## 4. Chuẩn bị bản đồ

Để chạy phần Localization/Nav2 với world `maze`, chép **`map.yaml` và `map.pgm`** đã tạo ở bài 4 vào:

```text
tb4_assignment5/maps/
```

Bảo đảm thuộc tính `image` trong `map.yaml` trỏ đúng tới `map.pgm`. Khi chỉ chạy YOLO mà không bật Nav2 thì không cần bản đồ này.

## 5. Build package

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select tb4_assignment5
source install/setup.bash
```

Kiểm tra package đã được ROS 2 nhận diện:

```bash
ros2 pkg prefix tb4_assignment5
```

**Lưu ý:** Trước khi sử dụng chức năng điều hướng 3 điểm, cần điền tọa độ thực tế vào `config/goals.yaml`. Lần đầu chạy YOLO có thể tự tải model `yolov8n.pt`.
