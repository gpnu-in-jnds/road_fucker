# road_finder

面向地瓜派 RDK X5 的 C++17 / OpenCV 4 道路识别程序。相机固定朝前下方拍摄，输入为
1280×720 BGR 图像，默认缩放至 640×360 处理。程序输出道路两侧边线、中线、偏差、
直道、左右 90° 转弯、左右分支及 T 字路口；不控制电机，也不替上层选择 T 字路口的方向。

## 构建

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

需要 OpenCV 4 的 `core`、`imgproc`、`videoio`、`highgui`、`imgcodecs` 模块，
无需 contrib。开发机也可使用 OpenCV 5 构建。无显示器时加 `--no-display`。

源码按职责分为四块：`src/main.cpp`（参数解析与主循环）、`src/camera.cpp`（打开 V4L2 设备、
取帧与尺寸归一化）、`src/preview.cpp`（预览窗口、叠加绘制、FPS 与关窗检测）、
`src/road_detector.cpp`（检测算法）。`ctest` 覆盖检测算法、预览关窗状态机与输入源分类。

## 运行

```sh
./build/road_finder                    # 默认 /dev/video0，打开预览窗口，不输出 JSON
./build/road_finder --source /dev/video2
./build/road_finder --source 0 --camera-fourcc MJPG --camera-fps 30
./build/road_finder --json             # 需要时逐帧输出 JSON
./build/road_finder --no-display --json
./build/road_finder --source frame.png --overlay /tmp/overlay.png --mask /tmp/mask.png
```

不带任何参数时，程序打开 `/dev/video0`（V4L2、MJPG、配置中的 1280×720 与 30 FPS），
逐帧检测并弹出预览窗口。`--source` 只接受摄像头序号（`0`）、`/dev/videoN` 路径或图片
（`.png/.jpg/.jpeg/.bmp/.webp`）；视频文件与 GStreamer 管道不再支持，会直接报错。
`--display`、`--no-json` 仍作为兼容别名被接受（分别等价于默认行为）。

两种摄像头来源都会请求配置中的 1280×720 和默认的 30 FPS。`/dev/videoN` 默认先选择 MJPG，
以支持常见 UVC 摄像头的 720p/30 FPS 模式；可用 `--camera-fourcc`、`--camera-fps` 覆盖。
程序会打印驱动协商后的分辨率和帧率。若摄像头只返回其他 16:9 分辨率，程序会保持宽高比
缩放至配置尺寸后检测，并在 stderr 提示；其他宽高比会报出实际尺寸。图片必须符合配置尺寸。
先用 `v4l2-ctl -d /dev/video2 --list-formats-ext` 确认设备支持的格式、分辨率和帧率。
若出现 `VIDIOC_QBUF` 错误或无法取帧，先用
`v4l2-ctl -d /dev/video2 --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=30 --stream-mmap --stream-count=10`
独立测试相同的设备模式；
若该命令也失败，则需检查摄像头驱动、USB 连接或设备占用。

`--overlay`、`--mask` 只用于单张图片。视频预览逐帧显示最终拟合的左边线
（青色）、右边线（橙色）和中线（绿色），以及当前路况、置信度和实际显示帧率。
预览窗口按 1280×720 画面尺寸打开，并保持原始宽高比；仍可手动调整窗口大小。
画面上的黄色细框是当前生效的检测窗口，含义与调法见[配置与现场标定](#配置与现场标定)。
路况标签包括 `STRAIGHT`、`LEFT TURN`、`RIGHT TURN`、`LEFT BRANCH`、
`RIGHT BRANCH`、`T JUNCTION`、`UNKNOWN` 和 `LOST`；`LOST` 表示本帧没有
有效的循迹几何结果，并不表示道路终止。FPS 统计完整的读取、处理、输出与显示循环，
每秒更新一次，开始时显示 `MEASURING`。
按 Esc 或 Q、关闭窗口时结束预览，并在 stderr 打印实际停止原因。
`--no-display` 关闭预览窗口，供无显示器的场合使用，与 `--json` 组合即可无界面运行。
在 GTK 后端的 OpenCV（RDK X5、Ubuntu 自带 OpenCV 4）上，highgui 没有实现用于检测
关窗的 `WND_PROP_VISIBLE` 属性，点窗口关闭按钮后 OpenCV 会重新创建预览窗口；
请用 Esc、Q 或 Ctrl+C 结束预览，程序会在若干帧后提示一次该限制。

加 `--json` 时，每帧在 stdout 输出一行 JSON；处理耗时和错误始终写入 stderr。字段包含
`timestamp_ms`、`valid`、`type`、`directions`、`confidence`、
`lateral_error`、`heading_error_rad`、`left_boundary`、`right_boundary`、
`centerline`。边线和中线坐标均基于原始 720p 图像。横向偏差按半幅道路宽度归一化，
目标中线在车体右侧时为正；方向偏差向右为正。`confidence` 表示道路几何结果的可信度；
路口仍在连续帧确认期间时，可能同时出现有效中线和 `UNKNOWN` 路口类型。

## 性能与帧率

程序每秒在 stderr 打印一行分阶段耗时，用来判断时间到底花在哪里：

```
fps 5.2 | read 12.1 ms | detect 41.3 ms | preview 122.5 ms | frames 26
```

`read` 是等摄像头出帧（含 MJPG 解码），`detect` 是检测算法，`preview` 是叠加绘制 + `imshow` +
`waitKey`。`--benchmark <frames>` 跑固定帧数后打印各阶段均值并退出，便于前后对比：

```sh
./build/road_finder --benchmark 300 --no-display   # 只测检测吞吐（也可以配图片源）
./build/road_finder --benchmark 300                # 含预览，两行相减就是显示开销
```

按结果对症下药：

| 耗时最大的一段 | 处理办法 |
| --- | --- |
| `read` | 多半是摄像头模式而不是程序：用 `v4l2-ctl -d /dev/video0 --list-formats-ext` 确认 720p 下有 MJPG/30fps；启动日志里的 `camera negotiated ... FPS` 明显低于请求时程序会提示 |
| `detect` | 收紧 `roi`、降低 `process_width`（像素阈值需一起复评）；检测已默认多线程，可用 `--threads 1` 对比 |
| `preview` | 用 `--preview-scale 0.5` 缩小预览，或用 `--preview-fps 15` 限制重绘频率（检测仍逐帧） |

相关选项（全部可选，不加参数时行为与之前一致）：

- `--preview-scale <0.1..1.0>`：预览画面与窗口按比例缩小。OpenCV 的 GTK 后端每帧都要做一次
  全帧 BGR→RGB 转换，窗口小于画面时还要再做一次 `INTER_AREA` 缩放，最后用 cairo 软件绘制，
  所以显示开销基本与像素数成正比；RDK X5 上建议 `0.5`（像素数降到 1/4）。
- `--preview-fps <Hz>`：限制预览重绘频率，检测与 JSON 输出不受影响（按键、关窗检测仍每帧处理）。
- `--threads <N>`：设置 OpenCV 线程数；默认用 CPU 核数（X5 为 4 核），`--threads 1` 关闭并行。
  并行只用于彼此独立的行扫描与骨架化单趟，输出与单线程完全一致。
- `--benchmark <frames>`：跑固定帧数后打印统计并退出，方便在目标机上量化每次调整。

## 故障排查

### 预览窗口黑屏且一帧后退出

旧版本用 `cv::getWindowProperty(..., cv::WND_PROP_VISIBLE) < 1` 判断用户是否关闭了窗口。
GTK 后端的 highgui 没有实现该属性（[opencv/opencv#25346](https://github.com/opencv/opencv/issues/25346)），
OpenCV 会把后端返回的 NaN 映射成 -1，于是 `-1 < 1` 在第一帧后就让预览循环 `break`：
窗口还没绘制就退出，表现为黑屏、一帧后静默退出（退出码 0、stderr 无错误）。
现在只有“曾经报告过可见、随后报告不可见”才算关闭；`-1`、`NaN` 或探测抛异常一律视为
“后端无法作答”，预览继续运行。停止时程序会打印 `preview stopped by key` 或
`preview window closed`，不再静默退出。

查看当前使用的 highgui 后端：

```sh
OPENCV_LOG_LEVEL=DEBUG ./build/road_finder --no-display 2>&1 | grep -E 'UI:|backend'
```

### 找不到 freedesktop.org 服务之类的警告

打开预览窗口时可能出现 `freedesktop.org`、`org.freedesktop.*` 服务的警告。
这是 GTK/GLib 在目标机的 XFCE 会话里找不到会话 D-Bus 服务（例如 desktop portal）时的
环境提示，与摄像头、检测算法和预览窗口无关，也不会导致程序退出。需要确认完整文本时：

```sh
G_MESSAGES_DEBUG=all ./build/road_finder --no-display 2>&1 | tee /tmp/road_finder.log
```

若文本指向 `org.freedesktop.portal.Desktop`，可在镜像里安装并启用 `xdg-desktop-portal`
与 `xdg-desktop-portal-gtk`，或直接在 XFCE 会话内的终端启动程序，使
`DBUS_SESSION_BUS_ADDRESS` 存在。无显示器的 `--no-display` 运行不受影响。

## 配置与现场标定

[配置文件](config/road_finder.yaml) 里只有两个几何参数决定“看哪里、看多大”：

- `process_width`：处理分辨率**只写宽度**，高度按输入画面宽高比自动推导（`640` → 640×360），
  因此不会出现输入/处理宽高比不一致这种错误。所有像素单位的阈值
  （`edge_line_residual_px`、`morphology_pixels` 等）都以这张处理图为单位。
- `roi: [x, y, 宽, 高]`：检测窗口，单位是**原始画面像素**，与预览画面、`--overlay` 导出图、
  `perspective_src/perspective_dst` 使用同一坐标系；对着画面量出道路的大致范围直接填即可。
  窗口必须延伸到画面底部附近（≥80% 高度），因为道路轮廓是从窗口底部向上生长的。

不用靠心算确认：启动时 stderr 会打印一行解析结果，例如

```
geometry: input 1280x720, process 640x360, roi [26,86,1229,612] input px -> [13,43,614,306] process px
```

并且预览窗口与 `--overlay` 导出图上会用**黄色细框**画出实际生效的检测窗口；开启透视时，
框是“透视图中实际窗口”反变换回原始画面的形状。旧的 `process_height` 与 `roi_fraction`
已移除，配置文件里若还有这两个键，程序会直接报错并给出等价写法，例如
`roi_fraction: [0.02, 0.12, 0.96, 0.85]` → `roi: [26, 86, 1229, 612]`
（换算：`像素 = round(比例 × 输入宽/高)`）。

默认参数只是起点。先固定相机，把各类道路和不同光照下的画面保存成 1280×720 的 PNG
（用外部截图/抓帧工具即可），再用图片模式导出掩码和叠加图（见 `--overlay`、`--mask`），
据此调整检测窗口、HSV 范围、道路宽度和分支阈值。样图中的灰色道路与背景都偏低饱和度，
因此算法还用逐行色差和边缘约束，不能仅凭一组 HSV 阈值保证所有光照下的效果。

直道边线采用跨行稳健直线拟合。`edge_line_residual_px`、`edge_min_inlier_ratio` 和
`edge_min_span_fraction` 控制离群点容忍度、最少支持比例和纵向覆盖；像素阈值以
640×360 处理图为单位。右边界证据不足时，输出 `valid: false`，不提供中线偏差。

如需俯视变换，同时填写 `perspective_src` 与 `perspective_dst`，各为四个对应点，
按左上、右上、右下、左下排序，坐标以原始 1280×720 图像为准。程序会自动缩放标定点。
保持画面底部中央有道路，因为区域筛选从这里确认连通性。接入底盘控制前，应在 X5
实机上测量帧率，并用已标注图片检查误报、漏报及减速带遮挡时的 `UNKNOWN` 结果。
