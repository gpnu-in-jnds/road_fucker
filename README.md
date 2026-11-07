# road_fucker

面向地瓜派 RDK X5 的 C++17 / OpenCV 道路循迹程序。相机固定朝前下方拍摄，默认输入
1280×720 BGR，缩放到 640×360 处理。使用固定 HSV 阈值分割道路，可另配终点颜色补充拟合像素，沿 ROI 分块提取真实
中心点，再拟合一条直线，输出横向偏差、航向角和道路状态。

本轮实现位于 `reimpl/road-forward`。已取消弯道、路口分类、左右路口框、红色停止区，
以及自适应色差／亮度分割。旧枚举及其旧串口映射仅在头文件中以注释留存。

## 构建与测试

需要 CMake 3.16+、C++17 和 OpenCV 4 的 `core`、`imgproc`、`videoio`、`highgui`、
`imgcodecs` 模块。开发机也支持 OpenCV 5，额外使用其 `geometry` 模块。

```sh
git lfs pull
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

相机样图由 Git LFS 管理，必须恢复图片实体后才能运行完整测试。所有构建和测试产物均在
`build/` 中。测试覆盖中线几何、尽头面积门限、实测点数量、当前帧发布、固定 HSV、遮挡、
分辨率、并行、透视、预览、配置、串口编码和无显示 CLI；现场 roadend 样图采用单独配置，
旧路口照片仅用于场景回归。

实际可执行文件名是 `road_fucker`。不需要测试时设置 `-DBUILD_TESTING=OFF`。

## 运行

```sh
./build/road_fucker                         # /dev/video0 + /dev/ttyS1，无窗口
./build/road_fucker --source /dev/video2 --no-serial
./build/road_fucker --source 0 --camera-fourcc MJPG --camera-fps 30
./build/road_fucker --display --threads 4 --json
./build/road_fucker --display --preview-scale 0.5 --preview-fps 15
./build/road_fucker --display --full-process --no-serial
./build/road_fucker --source tests/fixtures/t_junction_live_720p.png --no-display --no-serial --json
./build/road_fucker --source tests/fixtures/t_junction_live_720p.png --no-serial --overlay build/overlay.png --mask build/mask.png
./build/road_fucker --no-display --no-serial --benchmark 300
```

`--source` 接受相机序号、`/dev/videoN` 或 `.png/.jpg/.jpeg/.bmp/.webp` 图片；不支持视频文件
和 GStreamer 管道。图片尺寸须与配置一致。相机默认请求 MJPG、30 FPS；协商得到其他相同
宽高比的尺寸时会提示并缩放，其他宽高比报错。

默认关闭预览与 JSON。相机来源默认发送串口，图片来源只有显式指定 `--serial-device` 才发送。
`--no-serial` 关闭串口，`--json` 启用逐帧结果。`--overlay` 和 `--mask` 只用于图片来源。
`--threads N` 使用 N 个工作线程及 N×6 个条带，条带覆盖整个循迹 ROI。

预览按 Esc／Q 退出，相机预览按 S 保存原图、掩码、叠加图到 `build/captures/`。
部分 highgui 后端不支持窗口关闭状态查询，程序会提示改用 Esc／Q。
无显示器时使用 `--no-display`；它不调用 GUI。

## 三态与道路尽头

| C++ 枚举 | 数值／串口类型 | JSON／预览 | 含义 |
| --- | --- | --- | --- |
| `RoadType::Straight` | 0 | `STRAIGHT` | 当前有中线，尚未满足尽头的两个条件 |
| `RoadType::RoadEnd` | 1 | `ROAD_END` | 至少两个保留的实测质心可拟合直线，且尽头框内非 mask 占比 ≥80% |
| `RoadType::Unknown` | 2 | `UNKNOWN` | 本帧没有可用中线 |

中线将循迹 ROI 划分为 `threads × 6` 个水平检测条。每条先计算全部非零 mask 像素的
外接矩形，只有 mask 面积严格大于该矩形面积的 60% 时，才取全部 mask 像素的几何质心
作为候选点；恰好 60% 不取点，空 mask 也不取点。完整窄路可达到 100% 填充率。
默认启用中线点过滤：四个及以上候选点，用不同高度的点对生成 `x = a*y + b`，按
同一高度的横向残差 `abs(x - (a*y+b))` 找到多数点支持的直线。阈值为处理图像宽度的
2.5%，640 像素宽时为 16 像素，阈值边界包含在内。使用处理/透视视图坐标判断，
不按点离画面中心的远近判断，也不引入道路双边界仲裁。

候选优先级依次为支持点数、较小均方残差、较大纵向跨度，完全相同时保留固定点对顺序。
小点集枚举全部点对；超过 512 对时均匀采样点对索引，至多评估 512 个候选，结果可重复。
保留点必须严格超过原候选点数的一半且至少三个；对保留点做最小二乘拟合，再移除超限点并
重拟合，直到集合稳定。重拟合后也必须满足上述数量条件，否则输出 Unknown，所有实测点
均不参与拟合。原始质心仍完整保留用于诊断和黑点显示。

仅有一至三个候选点时不进行离群判断，诊断标记 `insufficient_points`；单点使用穿过该质心的
竖直线，该兜底竖线不满足 RoadEnd 的两个实测点要求。几何只使用当前帧，过滤无历史状态。
`enabled: 0` 恢复原全部点最小二乘拟合及原始底部点归一化偏移，便于现场对比。
该过滤针对少数离散误检；大量背景像素形成彼此一致的错误质心时，仍需修正道路分割。

RoadEnd 只检查两个条件：

1. 当前至少两个检测条的保留实测质心能够拟合直线，剔除点不计数。
2. `road_end_roi` 内合并道路掩码（`road_hsv` 与可选 `road_end_hsv` 的并集）的零值像素数占有效框面积的比例 **大于或等于 80%**。

条件满足就在当前帧发布，图片和视频行为一致，不连续确认、不锁存状态。桥接像素不作为
道路 mask；道路宽度、底部连接、横向边界、背景颜色、远处道路延续及遮挡诊断均不追加否决条件。
因此，这个规则也可能把框内足够大的分割缺失或遮挡判为 RoadEnd，它表达的是上述面积规则。
无中线时，即使框内全部为空也输出 Unknown；有单点兜底中线时输出 Straight。
`raw_type` 与 `type` 一致，`valid` 表示当前帧是否生成中线；RoadEnd 必须带有有效拟合几何。

尽头框用原始输入像素配置，可以位于循迹 ROI 外。透视时只统计映射多边形中的有效画面像素，
不把黑色填充算作非 mask；框完全不可见时不触发。框的位置决定触发距离，需要在目标转弯点标定。

现场裁剪样图 [roadend.png](tests/fixtures/roadend.png) 使用
[专用配置](tests/fixtures/roadend.yaml) 保留其 640×480 比例。其框 `[160,268,320,64]` 的非 mask
占比约 82.63%，单线程时有 5 个实测质心，默认过滤保留 4 个，首帧触发；`t-conner.png`
对照帧保留全部 6 点，非 mask 占比约 0.068%，输出 Straight。裁剪关系未知，这组坐标不能
直接用于 1280×720 实机画面。

```sh
./build/road_fucker --source tests/fixtures/roadend.png --config tests/fixtures/roadend.yaml --no-display --no-serial --json --overlay build/roadend_overlay.png
```

## 串口与 JSON

串口保持 **115200、8N1**，帧头为 `FF 7A`，不增加校验字节、帧尾或有效标志。

| 状态 | 帧长度 | 字节序列 |
| --- | --- | --- |
| Straight | 7 | `FF 7A 00 offset_lo offset_hi angle_lo angle_hi` |
| RoadEnd | 7 | `FF 7A 01 offset_lo offset_hi angle_lo angle_hi` |
| Unknown | 3 | `FF 7A 02` |

偏移、角度均为小端有符号 int16，四舍五入并限制到 int16 范围。偏移沿用拟合中线最下方点：
`round(center_x × 128 / input_width − 64)`，向右为正；角度沿用航向角的整数度，向右为正。
`RoadEnd` 没有有效几何时两项填零，有几何时带当前测量；`Straight` 没有有效几何则编码成 Unknown。

**下位机必须同步更新类型解释。** 旧串口 `01` 为左转、`02` 为右转、`04` 为未知；现在分别为
尽头和未知，且新的 `02` 是 3 字节短帧。帧结构保留不代表状态语义兼容。

JSON 保留 `timestamp_ms`、`valid`、`type`、`raw_type`、`confidence`、
`classification_confidence`、`occlusion_detected`、`lateral_error`、`heading_error_rad`、
`left_boundary`、`right_boundary`、`centerline`，新增：

- `road_end_detected`：当前帧已满足两个尽头条件，立即发布。
- `road_end_coverage`：检测框有效像素中的非 mask 面积占比；没有足够实测质心时也报告该占比。
- `raw_point_count` / `fit_point_count`：过滤前实测点数、最终参与拟合的点数。
- `centerline_filter_status`：`no_points`（无点）、`disabled`（关闭）、`insufficient_points`
  （一至三个点，跳过过滤）、`accepted`（全部保留）、`filtered`（剔除了部分点）、
  `no_consensus`（没有满足条件的一致点集，无中线）。
- `fit_residual_px`：保留点相对最终直线的横向 RMS 残差，单位为处理图像像素；无中线时为 0，
  需结合 `valid` 判断，不能把无中线的 0 理解为高质量拟合。
- `scan_blocks`：每块包含 `found`、`fit_used` 和原始画面坐标 `midpoint`（无实测点为 `null`）。
  `found` 表示测到质心，`fit_used` 表示参与最终拟合；剔除不会删除原始质心。

删除 `left_junction_hits`、`right_junction_hits` 和 `directions`。
`raw_type` 与 `type` 均为当前帧的三态结果。
启用过滤时，中线、航向、残差、宽度统计仅使用保留点；JSON `lateral_error` 取最终拟合线在
最下方保留点高度处的横坐标，并按保留点的半道路宽度归一化，不等于串口的 128 像素参考偏移。
置信度保留“参与点数 / 全部分块数”的惩罚，剔除大量点不会仅因残差变小就获得满置信度。
`heading_error_rad` 单位为弧度。几何和边界数组使用原始输入画面坐标。

## 配置、光照与标定

配置见 [config/road_finder.yaml](config/road_finder.yaml)。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_width` / `input_height` | 1280 / 720 | 输入尺寸 |
| `process_width` | 640 | 高度按输入宽高比推导 |
| `roi` | `[0,360,1280,360]` | 循迹窗口，原始画面像素，须延伸到至少图高 80% |
| `road_hsv` | `[70,10,10,150,40,230]` | 全图共用跨零点 H 范围及一组 S/V |
| `road_end_hsv` | 省略，不启用 | 额外计入路面的终点颜色，同时用于中线拟合和尽头面积判定，六项格式与 `road_hsv` 相同 |
| `road_end_roi` | `[400,540,480,120]` | 原始画面的触发框，须位于输入图内，可独立于 `roi` |
| `min_road_width_fraction` / `max_road_width_fraction` | 0.06 / 0.85 | 仅用于遮挡诊断，不约束中线或 RoadEnd |
| `morphology_pixels` | 3 | 道路分割开运算核大小 |
| `centerline_filter.enabled` | 1 | 是否启用中线点过滤，支持 0/1 |
| `centerline_filter.max_residual_fraction` | 0.025 | 最大横向残差 / 处理图像宽度，有限值且在 `(0,1]` 内 |
| `white_s_max` / `white_v_min` | 45 / 170 | 白色候选范围 |
| `occlusion_max_gap_fraction` | 0.25 | 可关联白色间断的最大高度占 ROI 高度比例 |

```yaml
centerline_filter:
  enabled: 1
  max_residual_fraction: 0.025
```

省略配置时使用上述默认值。增大阈值会保留更多点，减小阈值会更严格；初值需在现场确认。

HSV 配置格式为 `[Hmin,Hmax,Smin,Smax,Vmin,Vmax]`，H 为 0–179、S/V 为 0–255。
每个通道独立解释两个端点，所有区间均包含端点：

- 前值小于后值：匹配两者之间的区间，例如 `[10,20]` 匹配 10–20。
- 前值大于后值：匹配 `[前值,通道最大值]` 与 `[0,后值]` 的并集。
  例如 H `[50,20]` 匹配 50–179 或 0–20，S/V `[200,40]` 匹配 200–255 或 0–40。
- 前值等于后值：仅匹配该值，例如 `[20,20]` 只匹配 20。

三个通道的条件取交集，像素必须同时满足 H、S、V。不会自动交换端点；端点越界仍会报错。

`road_end_hsv` 与减速带白色候选一样，在当前处理／透视画面上直接进行 HSV 范围匹配；
但其命中像素作为真实测量，与道路掩码取并集后参与分块质心计算，继续受 60% 填充率与
中线离群点规则约束，不生成虚拟点或桥接点。只在循迹 ROI 内提取拟合点，颜色检测不限于
`road_end_roi`。同一像素若同时命中白色候选，以可参与拟合的实测像素为准。

```yaml
# 仅为格式示例，需按现场终点颜色标定后使用；删除/注释此字段即可关闭。
road_end_hsv: [25, 45, 10, 150, 40, 230]
```

终点面积判定与中线拟合使用同一合并掩码：命中 `road_hsv` 或 `road_end_hsv` 的像素均计为路面，
已配置的紫色终点区域不会仅因不匹配普通道路颜色而被算成背景。无需新增配置字段。
`road_mask`、`--mask` 与普通预览红色覆盖展示该合并掩码；`road_end_coverage` 是它在尽头框有效区域中的零值占比，
透视时仍排除无效填充像素。真正的非道路面积达到 80% 且拟合证据足够时，仍触发 RoadEnd；
少量终点色像素只减少非道路面积，不会直接否决触发。全流程颜色面板用青色显示终点 HSV 命中区域，
其余道路像素为绿色。阈值过宽会把背景算成路面并影响尽头检测，应结合青色覆盖标定。

默认 `road_hsv: [70,10,10,150,40,230]` 表示 **H70–179 与 H0–10 的并集**，
同时要求 `10 ≤ S ≤ 150`、`40 ≤ V ≤ 230`。这样可保留 H 接近 0 的路面区域，并排除
现场观察到的 H25–45 尽头背景。不要改成 H0–179，否则该背景也可能被分成道路。
低 H 区间先取 0–10；现场调整时应继续避开 H25–45。两段共用 S/V，使用现场已标定值，
无需新增两套阈值；即使 H 命中，S/V 越界的像素仍会被排除。
通过半透明红色覆盖检查两类路面均被选中、尽头背景未被选中，再验证中线及尽头触发。
形态学用于修整小噪声，不能可靠补回大片被 H 范围排除的道路。
初始范围不保证适用于不同灯具、相机或无彩色道路，现场仍须标定。

可同时配置 `perspective_src` / `perspective_dst`，每组按 TL、TR、BR、BL 给出四个原图点。
尽头框按多边形映射，不用透视后的外接矩形代替真实统计范围；无效透视填充像素不提供证据。
`processMask` 接口接收处理／透视坐标系的道路掩码，执行相同的中线及 RoadEnd 两条件规则，
不提供 BGR 白色遮挡诊断及终点 HSV 扩展，传入掩码同时用于拟合和终点面积计算。

**取消光照补偿有条件可行。** 原代码没有直方图均衡或 CLAHE，本轮进一步删除了
`adaptive_dark` 自适应分割和中央／两侧独立阈值。保留的高斯去噪、形态学、遮挡判断不属于
光照均衡。固定 HSV 分割不要求均衡步骤，参见
[OpenCV HSV 阈值说明](https://docs.opencv.org/4.9.0/da/d97/tutorial_threshold_inRange.html)。

辅助灯应覆盖整个检测范围，并保证道路与背景在近远处仍可区分。相机支持时固定曝光与白平衡，
在最终灯具和曝光设置下重新采样 HSV；本程序不自动控制曝光。需额外拍摄灯光亮暗变化、
近距离热点、斜向反光、阴影、白色遮挡、远处终点和接近终点的原始图像／录像进行验收。
超出标定范围时不会自动切换到亮度自适应模式。

## 从旧配置迁移与现场验收

以下旧键兼容忽略，可以从现场配置删除：`geometry_min_span_fraction`、`geometry_smoothing_alpha`、
`road_end_min_coverage`、`road_end_confirm_frames`、`road_end_support_fraction`、`min_blob_area_reference`。
旧配置中的 90% 或连续 3 帧不会覆盖当前固定的 80%／立即发布规则。

以下旧参数均已移除，加载时明确报错，不能靠保留旧键切回原算法：

- `segmentation_mode`、`road_contrast_min`、`side_road_hsv`、`side_left_end_reference`、`side_right_begin_reference`。
- `left_marker_rois`、`right_marker_rois`、`marker_min_coverage`、`indicator_roi`、`indicator_min_area_reference`。
- `stop_roi`、`stop_hsv`、`stop_detection_enabled`、`stop_min_area_reference`。
- `classification_confirm_frames`、`classification_hold_frames`。当前三态仅按本帧判断。

更早的 `road_lab`、`stop_lab`、固定 `scan_bands`、单个 `left_marker_roi` / `right_marker_roi`、
`process_height` 和 `roi_fraction` 仍会报迁移错误。更新配置时以本分支随附 YAML 为起点。
改变输入尺寸或循迹窗口时，也要同步修改原始坐标下的 `road_end_roi`。

常规预览／`--overlay` 将道路二值掩码的白色区域（固定 HSV 阈值命中、经形态学处理）
以 35% 不透明度的红色填满，方便检查漏检；未命中区域保持原图，未被中线选中的命中区域也会显示。
掩码按处理分辨率和透视变换映射回原图，缩小预览也保持对齐。红色覆盖之上显示
左上角三态、绿色循迹 ROI 提示框、紫色保留实测点、绿色拟合中线和黄色拟合点，以及
尽头框：红色为未满足两个条件、绿色为本帧已发布尽头。`--full-process` 保留原图、道路／
遮挡掩码、分块采样、最终叠加四个面板。循迹 ROI 在普通预览、分块面板和最终面板中显示，
透视处理时沿逆映射后的多边形绘制；尽头框仍按检测结果显示红／绿色。常规预览不显示路口框、道路边线或 FPS。
未参与拟合的实测点以**黑色实心点和细白描边**显示，包括无一致点集时的全部实测点。
黑点最后绘制，覆盖中线及尽头框；缩放预览在最终分辨率绘制，黑点至少保留 2 像素半径。
普通预览、`--overlay`、`--full-process` 的分块与最终叠加面板使用一致的点状态。
性能信息仍可通过 stderr 的 `--benchmark` 阶段统计获取。

现场先确定道路 HSV，再在目标转弯点调整尽头框，最后用实际行驶录像验证首次触发位置、误报和
漏报。局部遮挡或反光若满足两个条件，也会按该面积规则发布 RoadEnd，应纳入现场验收。
RDK X5 性能请用上述 300 帧基准在相同相机／分辨率／线程／显示设置下记录；本次仅完成
开发机 OpenCV 5 的构建、现场裁剪样图及合成场景回归，尚无 RDK X5 实测性能和实车录像验收结果。

2026-11-05 开发机（x86_64、OpenCV 5、Release）点过滤迭代基准：`roadend.png` 配合其专用
配置、640×480 处理、每次 300 帧、无显示和串口。检测阶段均值如下，四舍五入到 0.1 ms；
这是一次前后对比，包含调度噪声，不等同于 RDK X5 的耗时或纯过滤函数开销。

| 线程 | 修改前 | 默认启用过滤后 |
| --- | --- | --- |
| 1 | 4.9 ms | 4.9 ms |
| 2 | 3.8 ms | 3.8 ms |
| 4 | 3.5 ms | 3.6 ms |
