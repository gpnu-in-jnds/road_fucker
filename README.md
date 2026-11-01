# road_fucker

面向地瓜派 RDK X5 的 C++17 / OpenCV 4 道路识别程序。相机固定朝前下方拍摄，输入为
1280×720 BGR 图像，默认缩放至 640×360 处理。默认配置面向“深色道路、浅色地面”，
优先估计当前帧道路与地面的色差，色差不足时使用逐行与整帧亮度分割；
从底部选择连续道路区域，也支持标定后的 LAB 颜色阈值。
六个横向区域用于拟合道路方向，路口根据入口、横向出口和前方封闭区域分类。
路况分类与中线几何独立：能识别 T 字路口但无法测量中线时，不生成虚假的偏移。

## 构建

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

需要 OpenCV 4 的 `core`、`imgproc`、`videoio`、`highgui`、`imgcodecs` 模块，
无需 contrib。开发机也可使用 OpenCV 5 构建。无显示器时加 `--no-display`。

近景 T 字路口回归样图为 `tests/fixtures/t_junction_close.png`（640×480）。
下列配置保留原图比例，可重现分类结果：

```sh
./build/road_fucker --source tests/fixtures/t_junction_close.png --config tests/fixtures/t_junction_close.yaml --no-display --no-serial --json
```

另一张横梁占满画面、入口只在底部两角可见的原始帧为
`tests/fixtures/t_junction_near_raw.png`（2026-11-01 10:01:49，640×480），
使用同一配置也应输出 `T_JUNCTION`，而不是 `LOST`。此时循迹几何可能不可用，
`valid: false` 不等于路口分类失败；已识别的左转、右转、T 字路口在无中线时仍发送
对应类型的 7 字节串口包，偏移和角度填 0。
另一张2026-11-01 11:11:28的接近路口原图保存在`tests/fixtures/t_junction_approach_raw.png`。
该图单帧可识别T，但旧视频配置的3帧确认会使前两帧仍显示STRAIGHT，且中线不可用时发送UNKNOWN。
默认视频确认改为1帧，首次测得T就回传，与STM32的单包转向预备锁定一致；可通过配置恢复多帧确认。
检测回归覆盖三张原图及镜像、320/640处理宽度、720p缩放、直线后首帧T立即发布、单侧出口、反光路面和较暗地面。

实机1280×720样图`tests/fixtures/t_junction_live_720p.png`覆盖远端边界倾斜并贴近画面上沿的情况。
其两角残留的窄道路区域不能作为有效前向出口：T判定排除左右各5%的窄边缘后检查前向道路宽度，
仍须有连续的双侧出口及前方封闭证据。单侧转弯继续要求全宽封闭；内侧仍有可测宽度的前向支路不判T。

源码按职责分为：`src/main.cpp`（参数解析与主循环）、`src/camera.cpp`（摄像头采集）、
`src/preview.cpp`（预览与叠加绘制）、`src/road_detector.cpp`（检测算法）和
`src/uart_sender.cpp`（XDK42 二进制道路包）。

## 运行

```sh
./build/road_fucker                    # 默认 /dev/video0 + /dev/ttyS1，打开预览窗口
./build/road_fucker --source /dev/video2
./build/road_fucker --source 0 --camera-fourcc MJPG --camera-fps 30
./build/road_fucker --json             # 需要时逐帧输出 JSON
./build/road_fucker --no-display --json
./build/road_fucker --serial-device /dev/ttyS1
./build/road_fucker --no-serial        # 关闭默认串口输出
./build/road_fucker --source frame.png --overlay /tmp/overlay.png --mask /tmp/mask.png
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
路况标签包括 `STRAIGHT`、`LEFT TURN`、`RIGHT TURN`、`T JUNCTION`、
`UNKNOWN` 和 `LOST`。识别到路口但缺少循迹几何时，仍显示路口类型，
并标记 `GEOMETRY: UNAVAILABLE`；类型也无法确定时才显示 `LOST`。
FPS 统计完整的读取、处理、输出与显示循环，
每秒更新一次，开始时显示 `MEASURING`。
摄像头预览中按 **S**，在 `build/captures/` 保存同一帧的 `<时间戳>_raw.png`（原始画面）、
`_mask.png`（分割掩码）、`_overlay.png`（叠加图）；拍屏带有摩尔纹，诊断时优先使用这些文件。
按 Esc 或 Q、关闭窗口时结束预览，并在 stderr 打印实际停止原因。
`--no-display` 关闭预览窗口，供无显示器的场合使用，与 `--json` 组合即可无界面运行。
在 GTK 后端的 OpenCV（RDK X5、Ubuntu 自带 OpenCV 4）上，highgui 没有实现用于检测
关窗的 `WND_PROP_VISIBLE` 属性，点窗口关闭按钮后 OpenCV 会重新创建预览窗口；
请用 Esc、Q 或 Ctrl+C 结束预览，程序会在若干帧后提示一次该限制。

加 `--json` 时，每帧在 stdout 输出一行 JSON；处理耗时和错误始终写入 stderr。字段包含
`timestamp_ms`、`valid`、`type`、`raw_type`、`classification_confidence`、`directions`、`confidence`、
`lateral_error`、`heading_error_rad`、`left_boundary`、`right_boundary`、
`centerline`。边线和中线坐标均基于原始 720p 图像。横向偏差按半幅道路宽度归一化，
目标中线在车体右侧时为正；方向偏差向右为正。`confidence` 表示道路几何结果的可信度；
`classification_confidence` 表示类型证据，`raw_type` 是多帧确认前的单帧类型。`valid` 始终只表示
中线几何是否可用：可能出现 `valid: false, type: "T_JUNCTION"`，此时边线和中线数组为空，
偏移与角度不可用于循迹。停止区识别默认关闭；启用后命中停止颜色会立即清除已确认类型。

实时摄像头默认使用 `/dev/video0`，向 `/dev/ttyS1` 发送 115200、8N1 的道路帧。可用 `--serial-device`
覆盖设备路径，或用 `--no-serial` 关闭串口。图片源默认不发送，显式指定
`--serial-device` 后才发送一次。串口发送失败会报错退出；UART 二进制数据不会写入
stdout，`--json` 输出仍保持为逐帧 JSON。

串口包格式与 STM32 XDK42 接收端一致：已知道路为
`FF 7A type offset_x_le16 angle_le16`（7 字节），UNKNOWN 为 `FF 7A 04`（3 字节）。
类型值 0/1/2/3 依次表示直行/左转/右转/T 字路口。偏移使用 128 像素参考坐标，中心为 64、
右偏为正；角度以度为单位，竖直为 0、左偏为负、右偏为正。协议不支持的分支类型按
UNKNOWN 发送。已识别的左转、右转、T 字路口独立于中线有效性发送：没有有效中线时，
保留类型 1/2/3，偏移和角度均填 0，例如 T 字路口为 `FF 7A 03 00 00 00 00`。
这时 JSON 仍为 `valid: false`，预览仍标记 `GEOMETRY: UNAVAILABLE`；0 字段是协议
占位值，不代表测得居中。中线有效时继续发送实测偏移和角度。直行仍要求有效中线，
否则按 UNKNOWN 发送；停止区命中或分类为 UNKNOWN 时仍发送 `FF 7A 04`。

## 性能与帧率

程序每秒在 stderr 打印一行分阶段耗时，用来判断时间到底花在哪里：

```
fps 5.2 | read 12.1 ms | detect 41.3 ms | preview 122.5 ms | frames 26
```

`read` 是等摄像头出帧（含 MJPG 解码），`detect` 包含检测、UART/JSON 输出，`preview` 是叠加绘制 +
`imshow` + `waitKey`。`--benchmark <frames>` 跑固定帧数后打印各阶段均值并退出，便于前后对比：

```sh
./build/road_fucker --benchmark 300 --no-display   # 只测检测吞吐（也可以配图片源）
./build/road_fucker --benchmark 300                # 含预览，两行相减就是显示开销
```

按结果对症下药：

| 耗时最大的一段 | 处理办法 |
| --- | --- |
| `read` | 多半是摄像头模式而不是程序：用 `v4l2-ctl -d /dev/video0 --list-formats-ext` 确认 720p 下有 MJPG/30fps；启动日志里的 `camera negotiated ... FPS` 明显低于请求时程序会提示 |
| `detect` | 收紧 `roi`、降低 `process_width`（扫描带和 blob 面积会按处理图缩放） |
| `preview` | 用 `--preview-scale 0.5` 缩小预览，或用 `--preview-fps 15` 限制重绘频率（检测仍逐帧） |

相关选项：

- `--preview-scale <0.1..1.0>`：预览画面与窗口按比例缩小。OpenCV 的 GTK 后端每帧都要做一次
  全帧 BGR→RGB 转换，窗口小于画面时还要再做一次 `INTER_AREA` 缩放，最后用 cairo 软件绘制，
  所以显示开销基本与像素数成正比；RDK X5 上建议 `0.5`（像素数降到 1/4）。
- `--preview-fps <Hz>`：限制预览重绘频率，检测与 JSON 输出不受影响（按键、关窗检测仍每帧处理）。
- `--threads <N>`：设置 OpenCV 线程数；默认使用 OpenCV 的线程配置。
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
OPENCV_LOG_LEVEL=DEBUG ./build/road_fucker --no-display 2>&1 | grep -E 'UI:|backend'
```

### 找不到 freedesktop.org 服务之类的警告

打开预览窗口时可能出现 `freedesktop.org`、`org.freedesktop.*` 服务的警告。
这是 GTK/GLib 在目标机的 XFCE 会话里找不到会话 D-Bus 服务（例如 desktop portal）时的
环境提示，与摄像头、检测算法和预览窗口无关，也不会导致程序退出。需要确认完整文本时：

```sh
G_MESSAGES_DEBUG=all ./build/road_fucker --no-display 2>&1 | tee /tmp/road_finder.log
```

若文本指向 `org.freedesktop.portal.Desktop`，可在镜像里安装并启用 `xdg-desktop-portal`
与 `xdg-desktop-portal-gtk`，或直接在 XFCE 会话内的终端启动程序，使
`DBUS_SESSION_BUS_ADDRESS` 存在。无显示器的 `--no-display` 运行不受影响。

## 配置与现场标定

[配置文件](config/road_finder.yaml) 控制颜色分割和 ROI：

- `process_width`：处理分辨率只写宽度，高度按输入画面宽高比自动推导（`640` → 640×360）。
- `roi: [x, y, 宽, 高]`：循迹检测窗口，单位是**原始画面像素**，与预览画面、`--overlay` 导出图、
  `perspective_src/perspective_dst` 使用同一坐标系；对着画面量出道路的大致范围直接填即可。
  窗口必须延伸到画面高度的 80% 以下。路口形态在相同横向范围内搜索整帧高度，
  可识别循迹窗口上方的横道；黄色框仍只表示循迹窗口。
- `segmentation_mode`：随附配置使用 `adaptive_dark`。从底部中央估计道路颜色，
  从较亮的顶部边缘和下方外侧区域估计地面颜色；色差足够时用 B−G、R−G 色差分割，
  避免反光路面变亮或阴影地面变暗造成单侧出口丢失，不固定道路颜色。
  色差不足时比较逐行与整帧灰度分布；一行铺满道路时参考整帧亮度，保留 T 字横道。
  适合深色道路、浅色地面；
  画面仍需保留可区分的道路和地面，完全单色的画面不能提供可靠路口证据。
  设置为 `lab` 可恢复固定颜色阈值；旧配置未填写此键时也使用 `lab`。
- `road_contrast_min`：自适应模式的最小灰度差，默认 `18`（0–255），
  也用于筛选比入口道路更亮的地面参考。缺少足够色差和亮度对比时不分割。
- `min_road_width_fraction` / `max_road_width_fraction`：候选道路宽度占处理图宽度的范围，
  默认 `0.06`–`0.85`，只限制循迹几何，排除细电线和过宽背景；路口横道允许铺满画面。
  T 字路口要求可见的下方入口、连通的左右出口，以及前方封闭区域；不能只凭宽色块判断。
  近景入口偏向画面一侧时，左右出口分别按剩余可见边缘距离判断，仍要求持续的出口证据，
  防止固定外扩像素要求把 T 字口误作单侧转弯。
  前方封闭需要可通行道路在上方连续消失；道路偏离底部中线、渐宽或仍延伸到画面顶端，
  都不能单独作为 90° 转弯依据。分支上方重新出现持续的窄直行通道时，清除旧分支证据。
  近景入口仅露出最底部几行时，补充使用连续可见的两侧边界作为路口参考；
  没有两侧边界的整幅色块仍为 UNKNOWN，也不会凭少量入口行生成中线。
  双侧出口证据充分时，极细的连通色噪不算有效前向道路；此容错只用于 T 字口，
  单侧转弯仍要求完整封闭，以保留斜直道和渐宽直道的防误判约束。
- `classification_confirm_frames` / `classification_hold_frames`：视频默认 `1` 帧发布新类型，
  丢失时最多保持旧分类 `2` 帧；不保留失效中线。需要多帧消抖时可提高前者；图片模式不做多帧确认。
- `geometry_smoothing_alpha`：视频边线、中线、偏移与角度的平滑系数，默认 `0.35`，`1` 为不平滑。
  只平滑连续可测的几何，丢失时不复用旧中线；流中断超过 500 ms 或时间倒退会清除状态。
- `road_lab`：仅 `lab` 模式使用，格式为 OpenCV 8 位 LAB 的
  `[Lmin,Lmax,amin,amax,bmin,bmax]`，需要按相机原始帧标定。
- `scan_bands` 与顶部、左右、停止区 ROI 使用样例的 128×90 坐标，程序按处理图宽高缩放。
- blob 面积阈值使用 128×90 参考像素，程序按处理分辨率缩放。

不用靠心算确认：启动时 stderr 会打印一行解析结果，例如

```
geometry: input 1280x720, process 640x360, roi [26,86,1229,612] input px -> [13,43,614,306] process px
```

并且预览窗口与 `--overlay` 导出图上会用**黄色细框**画出实际生效的检测窗口；开启透视时，
框是“透视图中实际窗口”反变换回原始画面的形状。旧的 `process_height` 与 `roi_fraction`
已移除，配置文件里若还有这两个键，程序会直接报错并给出等价写法，例如
`roi_fraction: [0.02, 0.12, 0.96, 0.85]` → `roi: [26, 86, 1229, 612]`
（换算：`像素 = round(比例 × 输入宽/高)`）。

固定相机后保存不同路况和光照下的 1280×720 原始 PNG，用图片模式的 `--overlay`、
`--mask` 检查分割结果。拍摄显示器的照片只能辅助分析，不能替代相机颜色标定。
候选从底部向上选择，优先选择靠近画面中心的道路，并要求与底部区域连通、上下位置连续。
触及搜索窗口左右边界、只有单侧边缘的色块不提供中线；至少需要三个有效扫描带，
且最底部必须有效。几何置信度同时考虑扫描带数量、区域完整度和直线拟合残差，
与路况分类置信度分别报告，两者都不是校准后的正确率概率。
强反光、大片阴影或道路超出画面仍可能输出 `LOST`，应结合原始帧检查，避免只放宽阈值。
停止区识别默认关闭；启用时使用配置的颜色范围，样例 B 通道反向范围已按 [-45,-9] 修正。

如需俯视变换，同时填写 `perspective_src` 与 `perspective_dst`，各为四个对应点，
按左上、右上、右下、左下排序，坐标以原始 1280×720 图像为准。程序会自动缩放标定点。
保持六个扫描带覆盖道路，并在最底部扫描带保留有效道路区域。接入底盘控制前，应在 X5
实机上测量帧率，并用已标注图片检查误报、漏报及减速带遮挡时的 `UNKNOWN` 结果。
