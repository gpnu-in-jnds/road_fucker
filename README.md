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
无需 contrib。开发机也可使用 OpenCV 5 构建。无显示器时无需使用 `--display`。

## 运行

```sh
./build/road_finder --source 0 --config config/road_finder.yaml
./build/road_finder --source /dev/video2 --display --no-json
./build/road_finder --source /dev/video2 --camera-fourcc MJPG --camera-fps 30 --display --no-json
./build/road_finder --source 0 --display --no-json
./build/road_finder --source recording.mp4 --display --no-json
./build/road_finder --source frame.png --display --no-json
./build/road_finder --source frame.png --overlay /tmp/overlay.png --mask /tmp/mask.png
```

`--source` 接受摄像头序号、视频文件、图片，或以 `appsink` 结束的 GStreamer 管道。
摄像头也可用 `/dev/videoN` 路径；这两种摄像头来源都会请求配置中的 1280×720
和默认的 30 FPS。`/dev/videoN` 默认先选择 MJPG，以支持常见 UVC 摄像头的
720p/30 FPS 模式；可用 `--camera-fourcc`、`--camera-fps` 覆盖。程序会打印
驱动协商后的分辨率和帧率。
若摄像头只返回其他 16:9 分辨率，程序会保持宽高比缩放至配置尺寸后检测，
并在 stderr 提示；其他宽高比会报出实际尺寸。视频文件及图片仍须符合配置尺寸。
先用 `v4l2-ctl -d /dev/video2 --list-formats-ext` 确认设备支持的格式、分辨率和帧率。
若出现 `VIDIOC_QBUF` 错误或无法取帧，先用
`v4l2-ctl -d /dev/video2 --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=30 --stream-mmap --stream-count=10`
独立测试相同的设备模式；
若该命令也失败，则需检查摄像头驱动、USB 连接或设备占用。
`--overlay`、`--mask` 只用于单张图片。视频预览逐帧显示最终拟合的左边线
（青色）、右边线（橙色）和中线（绿色），以及当前路况、置信度和实际显示帧率。
预览窗口按 1280×720 画面尺寸打开，并保持原始宽高比；仍可手动调整窗口大小。
`--no-json` 可在预览时关闭终端中的逐帧 JSON 输出，不影响画面和检测结果。
路况标签包括 `STRAIGHT`、`LEFT TURN`、`RIGHT TURN`、`LEFT BRANCH`、
`RIGHT BRANCH`、`T JUNCTION`、`UNKNOWN` 和 `LOST`；`LOST` 表示本帧没有
有效的循迹几何结果，并不表示道路终止。FPS 统计完整的读取、处理、输出与显示循环，
每秒更新一次，开始时显示 `MEASURING`，不是视频文件声明的帧率。
按 Esc 或 Q、关闭窗口、或播放到文件末尾时结束预览，并在 stderr 打印实际停止原因。
`--display` 需要图形桌面环境；无显示器时不加该参数仍可逐帧输出 JSON。
在 GTK 后端的 OpenCV（RDK X5、Ubuntu 自带 OpenCV 4）上，highgui 没有实现用于检测
关窗的 `WND_PROP_VISIBLE` 属性，点窗口关闭按钮后 OpenCV 会重新创建预览窗口；
请用 Esc、Q 或 Ctrl+C 结束预览，程序会在若干帧后提示一次该限制。

每帧在 stdout 输出一行 JSON，处理耗时和错误写入 stderr。字段包含
`timestamp_ms`、`valid`、`type`、`directions`、`confidence`、
`lateral_error`、`heading_error_rad`、`left_boundary`、`right_boundary`、
`centerline`。边线和中线坐标均基于原始 720p 图像。横向偏差按半幅道路宽度归一化，
目标中线在车体右侧时为正；方向偏差向右为正。`confidence` 表示道路几何结果的可信度；
路口仍在连续帧确认期间时，可能同时出现有效中线和 `UNKNOWN` 路口类型。

## 故障排查

### 预览窗口黑屏且一帧后退出

旧版本用 `cv::getWindowProperty(..., cv::WND_PROP_VISIBLE) < 1` 判断用户是否关闭了窗口。
GTK 后端的 highgui 没有实现该属性（[opencv/opencv#25346](https://github.com/opencv/opencv/issues/25346)），
OpenCV 会把后端返回的 NaN 映射成 -1，于是 `-1 < 1` 在第一帧后就让预览循环 `break`：
窗口还没绘制就退出，表现为黑屏、一帧后静默退出（退出码 0、stderr 无错误）。
现在只有“曾经报告过可见、随后报告不可见”才算关闭；`-1`、`NaN` 或探测抛异常一律视为
“后端无法作答”，预览继续运行。停止时程序会打印 `preview stopped by key`、
`preview window closed` 或 `end of video source` 之一，不再静默退出。

查看当前使用的 highgui 后端：

```sh
OPENCV_LOG_LEVEL=DEBUG ./build/road_finder --source /dev/video0 --display --no-json 2>&1 \
  | grep -E 'UI:|backend'
```

### 找不到 freedesktop.org 服务之类的警告

启动 `--display` 时可能出现 `freedesktop.org`、`org.freedesktop.*` 服务的警告。
这是 GTK/GLib 在目标机的 XFCE 会话里找不到会话 D-Bus 服务（例如 desktop portal）时的
环境提示，与摄像头、检测算法和预览窗口无关，也不会导致程序退出。需要确认完整文本时：

```sh
G_MESSAGES_DEBUG=all ./build/road_finder --source /dev/video0 --display --no-json 2>&1 \
  | tee /tmp/road_finder.log
```

若文本指向 `org.freedesktop.portal.Desktop`，可在镜像里安装并启用 `xdg-desktop-portal`
与 `xdg-desktop-portal-gtk`，或直接在 XFCE 会话内的终端启动程序，使
`DBUS_SESSION_BUS_ADDRESS` 存在。无显示器的 `--no-json` 运行不受影响。

## 现场标定

默认参数只是起点。先固定相机，录制各类道路和不同光照下的 720p 视频，使用图片模式
保存掩码和叠加图，再调整 [配置文件](config/road_finder.yaml) 中的 ROI、HSV 范围、
道路宽度和分支阈值。样图中的灰色道路与背景都偏低饱和度，因此算法还用逐行色差和
边缘约束，不能仅凭一组 HSV 阈值保证所有光照下的效果。

直道边线采用跨行稳健直线拟合。`edge_line_residual_px`、`edge_min_inlier_ratio` 和
`edge_min_span_fraction` 控制离群点容忍度、最少支持比例和纵向覆盖；像素阈值以
640×360 处理图为单位。右边界证据不足时，输出 `valid: false`，不提供中线偏差。

如需俯视变换，同时填写 `perspective_src` 与 `perspective_dst`，各为四个对应点，
按左上、右上、右下、左下排序，坐标以原始 1280×720 图像为准。程序会自动缩放标定点。
保持画面底部中央有道路，因为区域筛选从这里确认连通性。接入底盘控制前，应在 X5
实机上测量帧率，并用已标注视频检查误报、漏报及减速带遮挡时的 `UNKNOWN` 结果。
