# Bài thực hành 05 – TurtleBot 4: Localization, Navigation và YOLO

Package ROS 2 `tb4_assignment5` dành cho **Ubuntu 22.04 + ROS 2 Humble**. Bài thực hành gồm 4 phần: nạp bản đồ và định vị, lấy tọa độ mục tiêu, điều hướng tự động qua 3 goal, và nhận diện vật thể bằng YOLO.

## 1. Cài đặt

### 1.1. Yêu cầu

- Ubuntu 22.04 (có thể chạy qua WSL2) và ROS 2 Humble.
- TurtleBot 4 Simulator, Gazebo và RViz2.
- Bản đồ `map.yaml` và `map.pgm` đã tạo từ bài thực hành 4 (dùng cho phần AMCL/Nav2).
- Kết nối Internet khi cài thư viện và tải model YOLO lần đầu.

### 1.2. Cài đặt thư viện

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

### 1.3. Thêm package vào workspace

Đặt thư mục `tb4_assignment5` (chứa `package.xml`, `setup.py`, `launch/`, `config/`...) vào `~/ros2_ws/src/`.

Nếu tải từ GitHub:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <URL_REPOSITORY_BAI_5> tb4_assignment5
```

Thay `<URL_REPOSITORY_BAI_5>` bằng URL GitHub của bài 5. Nếu đã có thư mục package thì bỏ qua bước clone.

Cài dependencies của YOLO:

```bash
cd ~/ros2_ws/src/tb4_assignment5
python3 -m pip install --user -r requirements-yolo.txt
```

### 1.4. Chuẩn bị bản đồ và build

Chép hai file của bài 4 vào thư mục `tb4_assignment5/maps/`:

```text
tb4_assignment5/maps/map.yaml
tb4_assignment5/maps/map.pgm
```

Bảo đảm thuộc tính `image` trong `map.yaml` trỏ đúng tới `map.pgm`. Nếu package bài 4 đã được cài và source, launch cũng có thể tự tìm bản đồ từ package đó. **Phần YOLO chạy riêng không cần bản đồ.**

Build và source workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select tb4_assignment5
source install/setup.bash
```

Trong mỗi terminal mới, chạy `source ~/ros2_ws/install/setup.bash` trước khi sử dụng package.

## 2. Hướng dẫn chạy theo 4 phần của đề bài

### Phần 1 – Khởi chạy Simulation và nạp bản đồ (Localization)

```bash
ros2 launch tb4_assignment5 assignment5.launch.py \
  world:=maze navigate:=false yolo:=false
```

- Gazebo khởi chạy TurtleBot 4 trong `maze`; Map Server, AMCL và Nav2 sử dụng bản đồ bài 4.
- Trong RViz2, chọn **Fixed Frame = `map`**; thêm display **Map** với topic `/map` nếu cần.
- Dùng **2D Pose Estimate** để thiết lập vị trí và hướng ban đầu đúng với robot trong Gazebo.
- Kiểm tra `/scan` trùng khớp các bức tường trên bản đồ.

### Phần 2 – Khảo sát và lấy tọa độ 3 goal

Chạy với node ghi tọa độ (hoặc chạy lại lệnh dưới đây sau khi dừng phiên trước):

```bash
ros2 launch tb4_assignment5 assignment5.launch.py \
  world:=maze navigate:=false yolo:=false points:=true
```

Trong RViz2, chọn **Publish Point** rồi nhấp vào 3 vị trí trống, thuộc các khu vực khác nhau trên bản đồ. Tọa độ được in ở terminal và có thể xem qua `/clicked_point`.

Điền các giá trị thực tế vào `config/goals.yaml`:

```yaml
goals:
  - name: Goal 1
    x: null  # Thay bằng tọa độ x của Goal 1
    y: null  # Thay bằng tọa độ y của Goal 1
    yaw: 0.0
  - name: Goal 2
    x: null
    y: null
    yaw: 0.0
  - name: Goal 3
    x: null
    y: null
    yaw: 0.0
```

**Phải thay toàn bộ `x`, `y` đang là `null` bằng tọa độ đo được.** `yaw` là góc hướng đích theo radian. Không chọn goal nằm sát tường hoặc trên vật cản.

### Phần 3 – Navigation tự động qua 3 goal bằng Nav2

Sau khi cấu hình đủ 3 goal, chạy:

```bash
ros2 launch tb4_assignment5 assignment5.launch.py \
  world:=maze yolo:=false navigate:=true
```

- Đặt **2D Pose Estimate** trong RViz2 nếu chưa có Initial Pose chính xác.
- Node `navigate_goals` tự gửi Goal 1 → Goal 2 → Goal 3 bằng Nav2 Action; không dùng chuột đặt Nav2 Goal.
- Quan sát đường đi và chuyển động trong Gazebo/RViz2.

Nếu chạy trên WSL2 và muốn thử cấu hình Nav2 giảm tải của package:

```bash
ros2 launch tb4_assignment5 assignment5.launch.py \
  world:=maze yolo:=false navigate:=true \
  wsl_optimized:=true tf_health:=true
```

Chế độ WSL chỉ là tùy chọn tối ưu và theo dõi TF, **không bảo đảm khắc phục mọi lỗi localization/TF**.

### Phần 4 – Nhận diện vật thể bằng YOLO

Chạy Gazebo + RViz2 + YOLO, **không khởi chạy AMCL/Nav2**:

```bash
ros2 launch tb4_assignment5 assignment5.launch.py \
  world:=maze nav_stack:=false navigate:=false \
  yolo:=true rviz:=true points:=false
```

Nếu muốn thử môi trường `warehouse` thay vì `maze`, đổi `world:=warehouse` (chỉ dành cho demo YOLO riêng, không dùng bản đồ `maze` để định vị trong world này).

Trong RViz2:

1. Chọn **Fixed Frame = `odom`** khi không chạy AMCL.
2. Thêm display **Image** và chọn topic `/yolo/annotated_image`.
3. Hướng camera vào vật thể thuộc các lớp YOLO hỗ trợ, ví dụ `person`, `chair`, `car`.

Kiểm tra các topic:

```bash
ros2 topic list | grep -E 'oakd|yolo'
ros2 topic hz /oakd/rgb/preview/image_raw
ros2 topic hz /yolo/annotated_image
ros2 topic echo /yolo/detections
```

Muốn bật cửa sổ OpenCV riêng, thêm `show_window:=true` vào lệnh launch. Lần đầu có thể cần tải file trọng số `yolov8n.pt`.

**Chạy đầy đủ bài 5 (Navigation + YOLO đồng thời):** sau khi đã có bản đồ và 3 goal hợp lệ, chạy:

```bash
ros2 launch tb4_assignment5 assignment5.launch.py
```

Thiết lập Initial Pose bằng **2D Pose Estimate**; robot sẽ tự điều hướng qua 3 goal trong khi YOLO xử lý ảnh camera.
