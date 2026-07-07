# AVP 泊车标注 JSON 重构实现 Spec

版本：`annotation_platform_reimplementation_spec_2026-07-01_master_v1`

适用基线：

- Git 分支：`master`
- 参考提交：`6e0bdb9 Update xfbagtools to 0.3.2`
- 输出 schema：`avp_annotation_schema_2026-06-23_v6`
- producer：`bag_to_annotation_2026-06-23_route_global`
- bag dump 工具：`xfbagtools 0.3.2`

本文面向标注平台技术栈重构实现。下游不需要直接部署当前 Python 脚本或 HTML 可视化工具，但重构后的实现必须保持本文定义的数据语义、字段契约、坐标系转换、时间对齐和质检要求。

## 1. 目标和边界

### 1.1 目标

重构实现需要完成以下能力：

1. 从 rosbag 或已 dump 的 `json_data/` 中读取 AVP 泊车场景所需消息。
2. 以 `/iflytek/fusion/objects` 为主帧时间轴，生成逐帧 annotation JSON。
3. 每帧输出固定业务字段：ego、agent、object、slot、lane、road item、occupancy、route、ego trajectory。
4. 将历史和未来轨迹统一转换到当前主帧 `ego(curr)` 坐标系。
5. 输出完整全局 route 文件 `ego_route_llh.json`，并让每帧 `data_route.route_index` 可回查完整 route。
6. 输出 manifest、summary、dependency 信息，支撑平台侧加载、回放、质检和问题定位。
7. 支持投影到相机、点云、雷达、地图等原始传感器数据做质量检查。

### 1.2 非目标

以下内容不作为本 Spec 的强制目标：

1. 复刻当前 HTML 可视化工具的 UI 或交互实现。
2. 复刻当前 Python 脚本的代码结构、函数名或目录结构。
3. 直接部署当前 `diffusion-annotation` 仓库。
4. 不在主分支 schema 中引入 `additional_info.position_llh` 输出。主分支当前只输出 `position.position_llh` 到 `ego_route_llh.json`。

## 2. 输入数据契约

### 2.1 rosbag topic

从 rosbag 直接生成时，至少需要 dump 以下 topic：

| topic | 目标 JSON | 必需性 | 用途 |
|---|---|---:|---|
| `/iflytek/fusion/objects` | `iflytek_fusion_objects.json` | 必需 | 主帧时间轴、agent、object |
| `/iflytek/localization/egomotion` | `iflytek_localization_egomotion.json` | 必需 | ego pose、速度、轨迹、route、坐标转换 |
| `/iflytek/vehicle_service` | `iflytek_vehicle_service.json` | 必需 | 方向盘、踏板、灯光、底盘状态质检字段 |
| `/iflytek/fusion/road_fusion` | `iflytek_fusion_road_fusion.json` | 必需 | lane line、reference line、stopline |
| `/iflytek/fusion/parking_slot` | `iflytek_fusion_parking_slot.json` | 条件必需 | 优先车位输入 |
| `/iflytek/fusion/spatial_parking_slot` | `iflytek_fusion_spatial_parking_slot.json` | 条件必需 | parking slot 缺失时 fallback |
| `/iflytek/fusion/occupancy/objects` | `iflytek_fusion_occupancy_objects.json` | 可选但推荐 | occupancy polygon |
| `/iflytek/mega/local_map` | `iflytek_mega_local_map.json` | 默认跳过 | section、lanelink、crosswalk-like road obstacle |

车位输入规则：

- `iflytek_fusion_parking_slot.json` 优先。
- 若 parking slot 缺失，允许尝试 `iflytek_fusion_spatial_parking_slot.json` fallback。
- 若两者都缺失，平台必须显式配置 allow missing slot，才允许输出空 `data_slot`。

### 2.2 dump 工具要求

当前主分支使用 `xfbagtools 0.3.2`：

- wheel 路径：`xfbagtools-skill-0.3.2/xfbagtools/assets/xfbagtools-0.3.2-py3-none-any.whl`
- Python 依赖：`lz4`、`av`、`Pillow`、`protobuf==3.20.3`、`opencv-python`、`ruamel.yaml`
- 默认使用 bag 内嵌消息定义自解析。
- 只有在老 bag 缺少内嵌定义时才需要 interface 目录。

平台如已有自研 bag 解析能力，可以不使用 `xfbagtools`，但导出的 JSON 字段结构必须与当前脚本消费的 `json_data/*.json` 等价。

## 3. 配置契约

### 3.1 默认配置

| 配置 | 默认值 | 含义 |
|---|---:|---|
| `AVP_SKIP_LOCAL_MAP` | `1` | 默认跳过 local map |
| `AVP_FRAME_SAMPLING` | `0` | 默认不做全局抽帧 |
| `AVP_FRAME_STRIDE` | `10` | 开启抽帧时默认每 10 个主帧输出 1 帧 |
| `AVP_DATA_ROUTE_STRIDE` | `10` | 每帧 `data_route` 默认 10 抽 1 |
| agent past window | `2s` | agent 和 ego 轨迹历史窗口 |
| agent future window | `10s` | agent 和 ego 轨迹未来窗口 |
| nearest tolerance | `80ms` | 多数感知帧最近邻匹配容差 |
| vehicle tolerance | `150ms` | vehicle service 最近邻匹配容差 |

### 3.2 抽帧规则

- 默认不抽帧，每个 `/iflytek/fusion/objects` 主帧输出一个 `frame_*.json`。
- 开启全局抽帧后，按 fusion objects 主帧序列每 `frame_stride` 帧输出一帧。
- 抽帧是全局策略，不是单字段策略。
- `label_ego_traj`、`data_agent`、`data_route` 等字段均基于输出主帧重新计算。

## 4. 输出目录和文件

重构实现建议输出以下结构：

```text
annotation_out/
├── frames/
│   ├── frame_000000.json
│   ├── frame_000001.json
│   └── ...
├── frames_manifest.json
├── annotation_index.json
├── summary.json
├── dependency_manifest.json
└── ego_route_llh.json
```

| 文件 | 要求 |
|---|---|
| `frames/frame_*.json` | 每个输出主帧一个 annotation JSON |
| `frames_manifest.json` | 保存帧号、源帧号、时间戳、相对路径 |
| `annotation_index.json` | 保存 schema、producer、字段列表、轨迹策略、依赖摘要 |
| `summary.json` | 保存帧数、时长、时间范围、首帧计数、耗时、notes |
| `dependency_manifest.json` | 保存字段到输入 topic/json 的依赖关系 |
| `ego_route_llh.json` | 保存完整全局 route 的经纬度、boot 位姿、速度、yaw/yaw_rate |

## 5. 顶层 frame JSON 契约

每个 `frame_*.json` 必须包含以下元信息：

| 字段 | 类型 | 含义 |
|---|---|---|
| `schema_version` | string | 固定为当前主分支 schema：`avp_annotation_schema_2026-06-23_v6` |
| `producer` | object | 生成器名称和版本 |
| `frame_index` | int | 输出帧序号，从 0 开始 |
| `source_frame_index` | int | 原始 fusion objects 主帧序号 |
| `frame_timestamp_us` | int | 当前主帧时间戳，单位 us |
| `output_frame_stride` | int | 输出抽帧步长，默认 1 |
| `frame_sampling_enabled` | bool | 是否启用全局抽帧 |
| `data_route_stride` | int | 每帧 route 稀疏步长 |

每个 `frame_*.json` 必须包含以下业务字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `data_ego_local_pose` | object | 当前主帧 ego 位姿和运动状态 |
| `data_roaditems` | object | stopline、crosswalk-like road item |
| `data_agent` | array | agent 白名单目标及窗口轨迹 |
| `data_object` | array | 非 agent 白名单目标，复用 agent-like schema |
| `data_slot` | array | 车位集合 |
| `data_ego_curr_status` | object | 当前主帧车辆状态 |
| `data_laneline` | array | reference line 和 lane boundary |
| `data_section` | object | local map section/lanelink，默认为空 |
| `data_occ` | array | occupancy polygon |
| `data_route` | object | 当前 ego 坐标系下的稀疏全局 route |
| `label_ego_traj` | object | 当前 ego 坐标系下 `[-2s, +10s]` ego 轨迹 |

## 6. 坐标系和转换

### 6.1 坐标系定义

当前输出中的二维几何点、目标轨迹、速度向量、车位角点、occupancy 多边形默认使用当前主帧 `ego(curr)` 坐标系：

| 轴 | 含义 |
|---|---|
| `+x` | 车辆前方 |
| `+y` | 车辆左方 |
| `+z` | 车辆上方 |

单位为米，右手坐标系。

`boot/global` 坐标系来自 egomotion：

- `position.position_boot`
- `orientation.quaternion_boot`
- `orientation.euler_boot.yaw`

### 6.2 点转换

boot/global 点转当前 ego 平面坐标的逻辑应等价于：

```text
dx = point_boot.x - ego_curr.position_boot.x
dy = point_boot.y - ego_curr.position_boot.y
x_curr =  cos(yaw_curr) * dx + sin(yaw_curr) * dy
y_curr = -sin(yaw_curr) * dx + cos(yaw_curr) * dy
```

`yaw_curr` 来自当前主帧插值得到的 egomotion yaw。

### 6.3 速度转换

原则：

- 目标速度和 ego 轨迹速度输出到当前主帧 `ego(curr)` 平面坐标系。
- 速度语义为对地速度，不是相对 ego 速度。
- `label_ego_traj.v` 是二维 `[vx, vy]`，不同于 `data_ego_curr_status.v` 的标量速度。

agent/object 当前规则：

- 读取 `common_info.velocity.x/y`。
- 按当前 ego yaw 旋转到 `ego(curr)` 平面坐标系。

ego trajectory 当前规则：

- 每个 egomotion 点先用该点 yaw 将 body velocity 转成 boot/global 对地速度。
- 再按当前主帧 yaw 转到 `ego(curr)` 平面坐标系。

### 6.4 yaw 转换

相对 yaw 输出范围必须归一化到 `[-pi, pi]`：

```text
yaw_relative = normalize_angle(yaw_source - yaw_curr)
```

### 6.5 待确认项

以下内容平台侧应保留可配置或在接口文档中继续标注待确认：

- ego/body 坐标原点：后轴中心、车辆几何中心、IMU 中心或其他。
- boot/global 原点定义和是否随定位重置漂移。
- 投影回传感器时，外参基准点与 ego 原点的严格对应关系。

## 7. 时间戳和对齐

### 7.1 主时钟

主帧时间轴固定为：

```text
iflytek_fusion_objects.json[].msg_header.stamp
```

输出 `frame_timestamp_us`、`reference_timestamp_us` 均以该主帧时间为基准。

### 7.2 单位

| 后缀 | 单位 | JS 注意事项 |
|---|---|---|
| `_us` | 微秒 | 当前量级可用 JS Number 精确表示 |
| `_ns` / `t_ns` | 纳秒 | 超过 JS safe integer，应使用字符串或 BigInt |
| `_ns_str` | 字符串纳秒 | 精确比较推荐使用 |
| `delta_t` | 秒 | 相对当前主帧时间，负数为历史，正数为未来 |

### 7.3 轨迹 current_index

除 `data_route` 外，轨迹对象必须满足：

```text
0 <= current_index < len(pos)
current_index 指向距离当前主帧时间戳最近的轨迹点
```

`data_route.current_index` 是例外：

- 每帧 `data_route` 使用同一组稀疏 `route_index`。
- 当前主帧不强制补入稀疏 route。
- 因此 `current_index=-1` 是合法值。
- 当前主帧在完整 route 中的位置看 `global_current_index`。

## 8. 字段生成规则

### 8.1 `data_ego_local_pose`

来源：`iflytek_localization_egomotion.json`

| 输出字段 | 规则 |
|---|---|
| `pos` | `position.position_boot.x/y` |
| `yaw` | `orientation.euler_boot.yaw` |
| `yaw_rate` | `angular_velocity.angvelocity_body.vz` |
| `v` | `velocity.velocity_body.vx/vy` 的速度范数 |
| `timestamp` | 当前主帧 `timestamp_us * 1000` |
| `timestamp_us` | 当前主帧时间戳 |
| `timestamp_ns_str` | 当前主帧纳秒字符串 |
| `pose_timestamp_us` | 插值 ego pose 时间戳，当前对齐主帧 |

注意：

- `pos` 是 boot/global 坐标，不是当前 ego 坐标。
- `v` 是标量速度范数。

### 8.2 `data_agent`

来源：

- `iflytek_fusion_objects.json`
- `iflytek_localization_egomotion.json`

agent 白名单 `common_info.type`：

```text
0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
18, 19, 20, 39, 40, 41, 42, 43, 47, 48, 49
```

窗口规则：

- 每帧包含当前主帧前 2 秒到后 10 秒内出现过的所有白名单 track。
- 每条 track 只保留窗口内真实观测点，不 padding，不插值。
- 每条 track 的所有点都转换到当前主帧 `ego(curr)`。

关键字段：

| 输出字段 | 规则 |
|---|---|
| `id` | 优先 `additional_info.track_id`，否则 `common_info.id` |
| `cls` | 直写 `common_info.type` |
| `x/y/z` | `current_index` 对应观测点的位置 |
| `vx/vy` | `current_index` 对应观测点速度，转到 `ego(curr)` |
| `size_x/size_y/size_z` | 来自 `common_info.shape.width/length/height` |
| `yaw` | 目标 heading 转到当前 ego 相对 yaw |
| `score` | `additional_info.confidence / 100` |
| `pos` | 窗口内全部观测点坐标 |
| `velocity` | 窗口内全部观测点速度 |
| `heading` | 窗口内全部观测点 yaw |
| `timestamp_us` | 窗口内全部观测点时间戳 |
| `t_ns_seq` / `t_ns_seq_str` | 轨迹点纳秒时间戳 |
| `delta_t` | 相对当前主帧的秒级时间差 |
| `current_index` | 距离当前主帧最近的点 |
| `has_exact_current_frame` | 是否存在精确同主帧时间戳观测 |
| `raw_pos` | 当前主帧及之前的历史观测子序列 |
| `valid_mask` | 每个轨迹点有效性 |

### 8.3 `data_object`

来源：`iflytek_fusion_objects.json`

规则：

- 包含不在 agent 白名单内的 fusion object。
- 字段结构复用 `data_agent`。
- `agent_mask=false`。
- `uses_agent_schema=true`。
- 下游不应把 `data_object` 当动态 agent 训练集合使用，除非另有业务配置。

### 8.4 `data_slot`

来源优先级：

1. `iflytek_fusion_parking_slot.json`
2. `iflytek_fusion_spatial_parking_slot.json`

规则：

- 车位独立写入 `data_slot`，不得追加到 `data_agent`。
- 常规 parking slot 存在时，使用 `parking_fusion_slot_lists`。
- spatial slot 只作为 parking slot 缺失时 fallback。
- `slot_corners` 转换到当前主帧 `ego(curr)`。
- `slot_corner_points_boot` 保留 boot/global 角点。
- `x/y` 为四角点中心。
- `size_x/size_y/yaw` 由角点几何估算。
- 车位为静态对象，`velocity` 固定为 `[0, 0]`。
- 为兼容旧 agent-like schema，`pos/raw_pos` 使用车位中心点合成序列。

待确认：

- 角点顺序是否固定。
- `yaw` 应表示长边方向还是入口方向。
- `slot_type`、`slot_side`、`slot_resource_type`、`allow_parking`、`fusion_source` 枚举表。

### 8.5 `data_ego_curr_status`

来源：

- `iflytek_localization_egomotion.json`
- `iflytek_vehicle_service.json`

规则：

| 输出字段 | 规则 |
|---|---|
| `v` | 优先 egomotion 速度范数，无 pose 时兜底 vehicle speed |
| `yaw_rate` | 优先 egomotion yaw_rate，无 pose 时兜底 vehicle service yaw_rate |
| `raw_vehicle_speed` | 最近 vehicle service `vehicle_speed` |
| `raw_vehicle_yaw_rate` | 最近 vehicle service `yaw_rate` |
| `steering.angle` | `steering_wheel_angle` |
| `throttle.pedal` | `accelerator_pedal_pos` |
| `ego_light` | `turn_switch_state` |
| `egolane_max_speed` | 固定 `255` |
| `lanes_topo` | 固定 `{}` |
| `map_type` | 固定 `8` |
| `ts` | 当前主帧纳秒时间戳 |

注意：

- `raw_vehicle_speed/raw_vehicle_yaw_rate` 是质检参考字段，不是主输出速度来源。
- `steering.angle` 单位和正方向仍需确认。

### 8.6 `data_laneline`

来源：`iflytek_fusion_road_fusion.json`

规则：

- 中心参考线来自 `lane_reference_line.virtual_lane_refline_points[].car_point`。
- 左右边界来自 `left_lane_boundary/right_lane_boundary.car_points`。
- `laneline_type` 对边界线直写上游 `boundary.type`。
- 中心参考线当前补 `laneline_type=0`。
- `pts_fixed_num` 为点集 `[x, y]`。
- `c0..c3` 来自 `poly_coefficient`，不足补 0。
- `start_x/end_x` 来自点集 x 范围。

LaneBoundaryType：

| 值 | 枚举名 | 含义 |
|---:|---|---|
| 0 | `LaneBoundaryType_MARKING_UNKNOWN` | 未知线型 |
| 1 | `LaneBoundaryType_MARKING_DASHED` | 虚线 |
| 2 | `LaneBoundaryType_MARKING_SOLID` | 实线 |
| 3 | `LaneBoundaryType_MARKING_SHORT_DASHED` | 短虚线 |
| 4 | `LaneBoundaryType_MARKING_DOUBLE_DASHED` | 双虚线 |
| 5 | `LaneBoundaryType_MARKING_DOUBLE_SOLID` | 双实线 |
| 6 | `LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID` | 左虚右实线 |
| 7 | `LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED` | 左实右虚线 |
| 8 | `LaneBoundaryType_MARKING_DECELERATION` | 减速线 |
| 9 | `LaneBoundaryType_MARKING_VIRTUAL` | 虚拟线 |
| 10 | `LaneBoundaryType_MARKING_WAITLINE` | 待转线 |
| 11 | `LaneBoundaryType_MARKING_DECELERATION_DASHED` | 减速虚线 |
| 12 | `LaneBoundaryType_MARKING_DECELERATION_SOLID` | 减速实线 |

### 8.7 `data_roaditems`

来源：

- stopline：`iflytek_fusion_road_fusion.json`
- crosswalk-like polygon：`iflytek_mega_local_map.json`，默认跳过

规则：

- `stoplines` 仅在 `stop_line.existence == true` 且存在 `car_points` 时输出。
- `crosswalks` 在默认 `AVP_SKIP_LOCAL_MAP=1` 时为空。
- 若启用 local map，历史逻辑将 `road_obstacle.type == 2` 写入 `crosswalks`，该语义仍需地图类型表确认。

### 8.8 `data_section`

来源：`iflytek_mega_local_map.json`

默认：

- `AVP_SKIP_LOCAL_MAP=1`
- `section=[]`
- `lanelink=[]`
- `section_link=[]`

启用 local map 时，当前输出主要是 road center 几何参考，不是完整拓扑。下游不应把它当高可靠拓扑图使用。

### 8.9 `data_occ`

来源：

- `iflytek_fusion_occupancy_objects.json`
- `iflytek_localization_egomotion.json`

规则：

| 输出字段 | 规则 |
|---|---|
| `id` | `common_occupancy_info.id` |
| `type` | `common_occupancy_info.type` |
| `track_id` | 优先 `additional_occupancy_info.track_id`，否则 `id` |
| `confidence` | `additional_occupancy_info.confidence / 100` |
| `polygon` | occupancy polygon 转到当前 `ego(curr)` |
| `center` | occupancy center 转到当前 `ego(curr)` |
| `point_count` | 输出 polygon 点数 |
| `visible_seg_num` | `additional_occupancy_info.visable_seg_num` |
| `timestamp_us` | 匹配到的 occupancy 原始帧时间戳 |

注意：

- `data_occ` 是占用区域，不是 freespace。
- freespace 如需输出，应由平台基于 ROI 与 occupancy polygon 另行构造。
- 当前转换只用 yaw 做地面平面转换，不把 pitch/roll 投影进 x/y。

### 8.10 `data_route`

来源：`iflytek_localization_egomotion.json`

规则：

- 表示完整全局 ego route 在当前主帧 `ego(curr)` 下的稀疏表达。
- 每帧使用完全相同的 `route_index` 序列。
- 默认 `data_route_stride=10`。
- 首个全局 route 点和最后一个全局 route 点必须保留。
- 当前帧点不强制补入。
- `current_index=-1` 是合法值。
- `global_current_index` 始终表示当前主帧在完整 `ego_route_llh.json` 中的位置。

关键字段：

| 输出字段 | 规则 |
|---|---|
| `pos` / `ego_routing` | 稀疏 route 点转到当前 ego |
| `v` | route 点对地速度转到当前 ego |
| `yaw` | route 点 yaw 相对当前 ego yaw |
| `yaw_rate` | route 点 yaw_rate |
| `timestamp_us` | route 点时间戳 |
| `timestamp_ns` / `timestamp_ns_str` | route 点纳秒时间戳 |
| `route_index` | 完整 route 索引 |
| `delta_t` | 相对当前主帧时间差 |
| `point_count` | 每帧稀疏 route 点数 |
| `global_route_point_count` | 完整 route 点数 |

### 8.11 `ego_route_llh.json`

来源：`iflytek_localization_egomotion.json`

主分支字段：

| 字段 | 含义 |
|---|---|
| `schema_version` | 当前 schema 版本 |
| `producer` | 生成器信息 |
| `source` | egomotion route 来源说明 |
| `coordinate` | WGS84 经纬高、boot pose、速度说明 |
| `output_frame_stride` | 输出抽帧步长 |
| `frame_sampling_enabled` | 是否启用抽帧 |
| `data_route_stride` | 每帧 route 稀疏步长 |
| `point_count` | 完整 route 点数 |
| `points[]` | 完整 route 点 |

`points[]` 字段：

| 字段 | 含义 |
|---|---|
| `route_index` | 完整 route 索引 |
| `source_frame_index` | 对应主帧源序号 |
| `timestamp_us/ns/ns_str` | route 参考时间 |
| `reference_timestamp_us/ns/ns_str` | 同 route 参考时间 |
| `pose_timestamp_us/ns/ns_str` | egomotion pose 时间 |
| `llh` | `position.position_llh` 序列化结果 |
| `longitude/latitude/height` | `llh` 快捷字段 |
| `position_boot` | boot/global 位置 |
| `velocity_body` | egomotion body 速度 |
| `velocity_boot` | boot/global 对地速度 |
| `rotation_boot` | boot quaternion |
| `yaw` / `yaw_boot` | boot yaw |
| `yaw_rate` | yaw rate |
| `v` | 标量速度 |
| `is_interpolated` | 是否插值 pose |
| `interpolation_source_timestamp_us` | 插值来源时间戳 |

主分支注意：

- `llh` 来自 `position.position_llh`。
- 主分支不输出 `additional_info.position_llh`。
- 如后续要输出 additional LLH，应提升 schema version，避免平台误判字段存在性。

### 8.12 `label_ego_traj`

来源：`iflytek_localization_egomotion.json`

规则：

- 每帧输出当前主帧前 2 秒到后 10 秒内的 egomotion 轨迹。
- `pos` 转到当前主帧 `ego(curr)`。
- `v` 为二维速度 `[vx, vy]`，转到当前主帧 `ego(curr)`。
- `yaw` 为相对当前主帧 yaw。
- `yaw_rate` 直写对应 egomotion yaw_rate。
- `delta_t` 相对当前主帧时间，负数历史、正数未来。
- `current_index` 指向距离当前主帧最近的 egomotion 点。

## 9. ObjectType 分类

### 9.1 agent 白名单

以下类型进入 `data_agent`：

| type | 枚举名 | 含义 |
|---:|---|---|
| 0 | `OBJECT_TYPE_UNKNOWN` | 未知障碍物 |
| 1 | `OBJECT_TYPE_UNKNOWN_MOVABLE` | 未知可移动障碍物 |
| 3 | `OBJECT_TYPE_COUPE` | 轿车 |
| 4 | `OBJECT_TYPE_MINIBUS` | 面包车 |
| 5 | `OBJECT_TYPE_VAN` | 厢式轿车 |
| 6 | `OBJECT_TYPE_BUS` | 大型客车 |
| 7 | `OBJECT_TYPE_TRUCK` | 卡车 |
| 8 | `OBJECT_TYPE_TRAILER` | 拖车 |
| 9 | `OBJECT_TYPE_BICYCLE` | 自行车 |
| 10 | `OBJECT_TYPE_MOTORCYCLE` | 摩托车 |
| 11 | `OBJECT_TYPE_TRICYCLE` | 三轮车 |
| 12 | `OBJECT_TYPE_PEDESTRIAN` | 行人 |
| 13 | `OBJECT_TYPE_ANIMAL` | 动物 |
| 18 | `OBJECT_TYPE_CYCLE_RIDING` | 人骑着自行车 |
| 19 | `OBJECT_TYPE_MOTORCYCLE_RIDING` | 人骑着摩托车 |
| 20 | `OBJECT_TYPE_TRICYCLE_RIDING` | 人骑着三轮车 |
| 39 | `OBJECT_TYPE_SPECIAL_VEHICLE` | 特殊车辆 |
| 40 | `OBJECT_TYPE_PICKUP` | 皮卡 |
| 41 | `OBJECT_TYPE_SUV` | SUV |
| 42 | `OBJECT_TYPE_MPV` | MPV |
| 43 | `OBJECT_TYPE_ENGINEERING_VEHICLE` | 工程车 |
| 47 | `OBJECT_TYPE_ADULT` | 成人 |
| 48 | `OBJECT_TYPE_TRAFFIC_POLICE` | 交警 |
| 49 | `OBJECT_TYPE_CHILD` | 儿童 |

### 9.2 object 和 occ

- 不在 agent 白名单内的 fusion object 输出到 `data_object`。
- `data_occ[].type` 直写 occupancy object 的 `common_occupancy_info.type`。
- `OBJECT_TYPE_OCC_NUM=64` 应视为数量上限，不建议作为真实目标类型消费。

## 10. 平台实现建议

### 10.1 推荐模块划分

平台重构时建议拆成以下逻辑模块：

| 模块 | 职责 |
|---|---|
| `BagDumpAdapter` | rosbag topic 列表、dump、json_data 产物确认 |
| `SourceLoader` | 流式或索引式加载各输入 JSON |
| `TimeIndex` | 按时间戳做最近邻、窗口查询、插值 |
| `EgoPoseService` | egomotion 插值、boot/body 转换、速度转换 |
| `FusionObjectBuilder` | `data_agent`、`data_object` 构造 |
| `SlotBuilder` | parking/spatial slot 解析和 fallback |
| `RoadBuilder` | laneline、roaditems、section 构造 |
| `OccupancyBuilder` | occupancy polygon 构造 |
| `RouteBuilder` | `data_route` 和 `ego_route_llh.json` 构造 |
| `FrameWriter` | frame JSON、manifest、summary、dependency 输出 |
| `Validator` | schema、数组长度、时间戳、route_index、current_index 校验 |

### 10.2 必须保持的行为

1. 主帧时钟必须来自 fusion objects。
2. 所有历史/未来轨迹必须表达在当前主帧 `ego(curr)` 下。
3. `data_agent` 不做固定长度 padding。
4. `data_agent` 每帧包含 `[current - 2s, current + 10s]` 窗口内出现过的所有白名单 track。
5. `label_ego_traj` 只输出 `[current - 2s, current + 10s]`。
6. `data_route` 每帧使用同一组稀疏 `route_index`，且包含首末点。
7. `ego_route_llh.json` 保存完整 route。
8. `data_slot` 独立字段输出，不混入 `data_agent`。
9. `data_ego_curr_status.v/yaw_rate` 优先使用 egomotion。
10. ns 时间戳必须提供字符串形式，避免 JS 精度问题。

### 10.3 可以按平台技术栈调整的内容

以下内容可重构，不影响语义：

- 文件读写方式：批量、流式、数据库中间表均可。
- 可视化实现：WebGL、Canvas、平台内置地图组件均可。
- JSON schema 定义形式：TypeScript interface、Pydantic、Protobuf、JSON Schema 均可。
- 任务调度方式：离线批处理、平台任务队列、服务化 API 均可。
- 依赖存储：软链接、对象存储、数据库元信息均可。

## 11. 校验和验收

### 11.1 结构校验

每批输出必须校验：

1. 所有 frame 文件可解析为 JSON。
2. 每帧顶层业务字段完整。
3. `frames_manifest.json` 中的帧数、路径和时间戳与实际 frame 文件一致。
4. `annotation_index.json`、`summary.json`、`dependency_manifest.json` 存在。
5. `ego_route_llh.json` 存在，且 `point_count == len(points)`。

### 11.2 轨迹校验

每个轨迹对象必须校验：

1. `pos`、`velocity`、`heading`、`timestamp_us`、`delta_t`、`valid_mask` 长度一致。
2. 除 `data_route` 外，`current_index` 在数组范围内。
3. `timestamp_us` 与 `t_ns_seq` 对应关系正确。
4. `delta_t[i] == (timestamp_us[i] - reference_timestamp_us) / 1e6`。
5. `raw_pos` 表示历史子序列，不应被描述为未转换原始坐标。

### 11.3 route 校验

必须校验：

1. 所有 frame 的 `data_route.route_index` 完全一致。
2. `route_index` 包含完整 route 的首点和末点。
3. `data_route.route_index[]` 均能在 `ego_route_llh.json.points[].route_index` 中找到。
4. `data_route.current_index=-1` 不应判错。
5. `global_current_index` 在完整 route 范围内。

### 11.4 BEV 质检

质检图层建议顺序：

1. Ego 和 `label_ego_traj`
2. `data_route`
3. `data_agent` / `data_object`
4. agent trajectory
5. `data_slot`
6. `data_occ`
7. `data_laneline`
8. `data_roaditems` / `data_section`

重点检查：

- 是否存在整体坐标偏移。
- agent box 是否贴合轨迹当前点。
- agent yaw 是否与 box 长边和运动方向一致。
- slot 四角点是否形成合理四边形。
- occupancy polygon 是否贴合障碍物、墙、柱等区域。
- route 与 ego trajectory 大方向是否一致。

### 11.5 投影到原始传感器质检

投影前必须确认：

- 原始传感器时间戳单位。
- 当前 frame `frame_timestamp_us`。
- 相机内参、外参、畸变参数。
- LiDAR/Radar 到 ego/body 的外参。
- ego/body 原点与外参基准点关系。

相机投影检查：

- agent box 是否包住真实目标主体。
- box 底边是否贴近地面接触位置。
- 车位角点是否贴合车位线交点。
- 车道线是否贴合图像标线。
- 多相机重叠区域中同一目标投影是否一致。

点云投影检查：

- 点云主体是否落在 box 内或边界附近。
- box 长宽高是否覆盖目标主体。
- box yaw 是否与点云轮廓一致。
- 静态障碍物是否连续帧稳定。

Radar/速度检查：

- `vx/vy` 方向应与 track 运动趋势一致。
- 静止目标速度应接近 0。
- 位置突变同时速度突变时，优先检查 track id 关联。

## 12. 问题等级

| 等级 | 说明 | 示例 |
|---|---|---|
| P0 | 阻塞 | JSON 无法加载、时间戳单位错误、坐标系整体错误 |
| P1 | 严重 | 大量 box 系统性偏移、yaw 系统性错误、slot 整体错位 |
| P2 | 一般 | 个别漏检误检、track 短暂跳变、个别车位尺寸异常 |
| P3 | 记录 | 远距离轻微偏移、遮挡局部不贴合、语义待确认 |

## 13. 明确待确认项

以下事项不阻塞按当前主分支重构，但应在平台 schema 或任务配置中显式标注：

1. ego/body 坐标原点。
2. boot/global 原点定义和定位重置行为。
3. slot 角点顺序和 yaw 语义。
4. `slot_type`、`slot_side`、`slot_resource_type`、`allow_parking`、`fusion_source` 枚举表。
5. `data_laneline.color`、`edge_type`、`lane_category` 枚举表。
6. `steering.angle` 单位和正方向。
7. `throttle.pedal` 数值范围。
8. `map_type=8` 和 `egolane_max_speed=255` 是否应继续保留。
9. local map 中 `road_obstacle.type == 2` 是否可定义为 crosswalk。
10. 是否需要在未来 schema 中增加 `additional_info.position_llh`。

## 14. 版本兼容说明

当前主分支为 `v6` 契约：

- `ego_route_llh.json.points[].llh` 使用 `position.position_llh`。
- 不包含 `additional_llh` 字段。
- `xfbagtools` 已更新到 0.3.2。

若平台后续吸收开发分支或新需求，应遵循：

1. 新增字段必须提升 schema version。
2. 不应改变已有字段语义而不改版本。
3. 新字段允许为 optional，但必须在 schema 中定义缺失语义。
4. 下游训练或渲染消费方必须按 `schema_version` 做兼容分支。

## 15. 参考文档

主分支参考文件：

- `README.md`
- `docs/AVP_OUTPUT_SCHEMA_CONTRACT.md`
- `docs/OBJECT_TYPE_MAPPING_data_agent_data_occ.md`
- `packages/visualizer_tool_latest/FIELD_RULES.md`
- `packages/visualizer_tool_latest/SCHEMA_CONTRACT.md`
- `packages/visualizer_tool_latest/ANNOTATION_RULES.md`
- `packages/visualizer_tool_latest/QA_VISUAL_INSPECTION_GUIDE.md`
- `scripts/build_json_data_all_sample_annotations_slots.py`
- `scripts/run_bag_to_annotation.py`
