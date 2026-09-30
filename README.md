# 机载 GNSS 完好性监测回放服务

本项目是面向机载 GNSS 接收机的地面回放原型：单历元完成加权单点最小二乘定位、卡方检测、故障排除和水平保护级（HPL）计算；跨历元维护卫星隔离/恢复与告警持续性状态。后端为 Go 1.22 + Echo，状态持久化在挂载的本地目录。

## 1. 包结构

| 包 | 职责 |
| --- | --- |
| `internal/coords` | WGS-84 ECEF/经纬高换算、当地 ENU 基向量、地心位置检查 |
| `internal/position` | 加权单点最小二乘、迭代停止、残差、HDOP、RAIM 斜率 |
| `internal/chisq` | 自实现中心/非中心卡方 CDF、分位数、漏检非中心参数反解；不使用统计库 |
| `internal/raim` | 单历元检测、逐星排除、唯一性确认、HPL、5/6 星规则 |
| `internal/profile` | 具名运行档、参数校验、内置航路/终端区/NPA 三档 |
| `internal/monitor` | 跨历元隔离/恢复状态机、告警持续性窗口、会话统计 |
| `internal/persistence` | 运行档和会话 JSON 文件持久化、原子写 |
| `internal/validation` | 请求字段校验，错误指向具体字段 |
| `internal/service` | 单历元/批量提交、批量原子性、会话锁 |
| `internal/httpapi` | Echo 路由与 HTTP 错误映射 |
| `cmd/integrity-server` | 服务入口 |
| `cmd/replay-metrics` | 固定种子策略对比分析工具 |
| `internal/sim` | 测试/文档用确定性卫星几何与高斯噪声生成器 |

## 2. 单历元处理

1. 输入每颗可见星的 ECEF 坐标、伪距和伪距误差标准差 `sigma`，以及概略 ECEF 位置。
2. 权重为 `1/sigma²`；状态为 `[Δx, Δy, Δz, Δb]`，其中 `Δb` 为接收机钟差（单位 m）。
3. 最多迭代 10 轮；位置修正范数小于 1 mm 时立即收敛。
4. 输出 ECEF、经纬高、钟差、实际迭代轮数、收敛标志和每颗可见星残差。
5. 加权残差平方和 `SSE = Σ(r_i/sigma_i)²` 服从自由度 `n-4` 的卡方分布。
6. 检测阈值为 `χ²(n-4, 1-Pfa)`。
7. `n<5` 时只定位并标记“完好性不可用”；`n>=5` 才能检测。
8. 检出后逐颗剔星重解：至少保留 5 颗、子集 SSE 通过对应阈值。**有且仅有一颗星的剔后解通过**才认为它是可唯一解释故障的星；否则报告排除失败。
9. `n<6` 时无剩余冗余做排除，如实报告无法排除。
10. HPL 使用最大水平斜率：

```text
HPL = max_i(slope_i) * sqrt(λ)
P(χ'²(n-4, λ) <= T) = Pmd
T   = χ²(n-4, 1-Pfa)
```

斜率由加权最小二乘协方差和残差投影矩阵计算，水平位移投影到 ECEF 位置处的 ENU 水平面。

## 3. 跨历元状态机

### 3.1 隔离与恢复

某颗星在一个历元被唯一排除后进入隔离。后续历元：

- 它不参与定位解算；
- 但仍逐历元把它单独加回，计算：
  - 它在加回解中的归一化残差 `|r/sigma|`；
  - 加回整体解的 SSE 和卡方检验；
- 两者同时通过，并且当前活动星座的快照本身可接受，才把该星的连续恢复计数加一；
- 任一条件失败，计数清零；
- 连续正常历元数达到运行档 `recovery_epochs` 后，才在下一历元前解除隔离；
- 该星不可见也会中断连续计数。

因此“刚剔除、下历元残差偶然回落又被拉回”的情况不会发生。

### 3.2 告警策略选型：快照输出 + 持续性窗口告警

本实现采用**组合方式**：

- **逐历元快照判定始终计算并返回**：定位状态、SSE、阈值、排除结果、HPL 与 `snapshot_unsafe` 不被隐藏；
- **外部告警状态由持续性窗口驱动**：连续 `alarm_epochs` 个不安全快照才拉起告警；连续 `clear_alarm_epochs` 个安全快照才撤警。

这样既保留单历元诊断的即时性，又避免单个异常历元造成告警闪烁。若需要旧式行为，可把两个窗口都配置为 1。

不安全快照包括：检测失败、无法唯一排除、解算失败、或 `HPL > HAL`。4 星“只能定位、完好性不可用”不等于故障，不单独拉起告警。

## 4. 运行档

运行档字段：

```json
{
  "name": "terminal",
  "probability_false_alarm": 0.00001,
  "probability_missed_detection": 0.001,
  "horizontal_alert_limit": 1852,
  "recovery_epochs": 3,
  "recovery_z_limit": 4,
  "alarm_epochs": 3,
  "clear_alarm_epochs": 3
}
```

内置三档：

| 档 | Pfa | Pmd | HAL |
| --- | ---: | ---: | ---: |
| `enroute` 航路 | `1e-5` | `1e-3` | 3704 m（2.0 nmi） |
| `terminal` 终端区 | `1e-5` | `1e-3` | 1852 m（1.0 nmi） |
| `npa` 非精密进近 | `1e-5` | `1e-3` | 556 m（0.3 nmi） |

统计量和 HAL 数值采用 RAIM 可用性筛查文献中常见的 Pfa=1/100000、Pmd=1/1000 表：RTCA DO-229D 附录 RAIM screening table，并参见 Walter 与 Enge 在 *Understanding GPS Principles and Applications* 第 5 章 “GPS RAIM: Detection of Failures”（1996）中的整理。持续性参数不是标准强制值，而是原型的可调参数；内置值采用 1 Hz 数据下 3 个历元，恢复窗口也为 3 个历元。

调用方可以通过 `POST /api/v1/profiles` 新建运行档；内置档不可覆盖。每个历元响应都包含 `profile_name`、当时卡方阈值、HPL 非中心参数、HAL 比较结果。

## 5. 回放数据中的策略影响

分析工具：

```bash
go run ./cmd/replay-metrics \
  -epochs 600 -satellites 5 -sigma 1 -hal 200 \
  -fault-start 201 -fault-end 300 -bias 120 \
  -false-burst-every 60 -false-burst-bias 180
```

固定种子 `20260930`，600 个 1 Hz 历元；201–300 历元为持续真实故障；每 60 个历元注入一次单历元多径式突偏。该场景有 5 颗可见星：单历元突偏可被检测但没有足够冗余完成排除，持续真实故障也会连续检测失败。

| 判定方式 | 首次告警历元 | 告警次数 | 误警次数 | 可用率 |
| --- | ---: | ---: | ---: | ---: |
| 逐历元快照（窗口=1） | 60 | 9 | 8 | 82.00% |
| 持续性窗口（告警/撤警=3） | 203 | 1 | 0 | 83.33% |

解读：

- **首次告警延迟**：持续性窗口对持续真实故障延迟 2 个历元（从故障开始的 201 到 203），在 1 Hz 下为 2 s；
- **误警次数**：所有单历元突偏均被 3 历元窗口滤除，误警由 8 次降为 0；
- **可用率**：快照策略在突偏后下一个历元立即撤警，但多次拉起/撤除导致告警占用较多；持续性策略虽然每次告警至少占 3 历元，却避免了 8 次无故障告警，本回放中可用率反而提高 1.33 个百分点；
- 当故障确实持续时，窗口只带来固定 `alarm_epochs-1` 个历元的有界延迟。

这是确定性仿真回放，不是飞行试验统计结论；真实飞行数据可用同一 HTTP API 回放并比较每会话统计量。

## 6. 会话、提交方式与持久化

- 创建会话时绑定一个运行档；会话 ID 随机生成。
- 支持：
  - `POST /sessions/:id/epochs`：逐历元实时提交；
  - `POST /sessions/:id/batches`：一次最多提交 3600 个历元。
- 同一批数据在两种提交方式下逐历元结果、隔离状态、告警状态和统计量完全一致；已有 `TestBatchAndIndividualSubmissionAreIdentical` 覆盖。
- 批量先完成全部校验并在状态克隆上处理，任一历元失败则不推进已持久化状态。
- 同一时间戳重复提交：返回冲突，不推进状态。
- 时间戳倒退：拒收。
- 状态以 JSON 原子写入挂载目录。重启后创建新的服务实例即可从最后时间戳续跑；`TestRestartResumeMatchesContinuousRun` 验证续跑结果与一口气跑完深度相等。
- 每会话统计：接受历元数、可用历元数、可用率、检出次数、告警次数、误警次数。

误警分类依赖可选测试字段 `faulty_prn`：若告警持续窗口内没有任何历元声明真实故障 PRN，则记为误警。生产飞行数据不传该字段时，统计仍按“未声明故障”处理。

## 7. HTTP API

### 健康检查

```http
GET /healthz
```

### 运行档

```http
GET    /api/v1/profiles
GET    /api/v1/profiles/:name
POST   /api/v1/profiles
```

### 会话

```http
POST   /api/v1/sessions
GET    /api/v1/sessions/:id
POST   /api/v1/sessions/:id/epochs
POST   /api/v1/sessions/:id/batches
```

单历元请求示例：

```json
{
  "timestamp": 1000,
  "satellites": [
    {
      "prn": 1,
      "satellite": {"x": 26825852.1, "y": 0.0, "z": 6643865.4},
      "pseudorange": 21500000.0,
      "sigma": 1.0
    }
  ],
  "initial_position": {"x": 6378137.0, "y": -420.0, "z": 260.0},
  "faulty_prn": null
}
```

批量请求：

```json
{
  "epochs": [ { "timestamp": 1, "...": "same epoch schema" } ]
}
```

## 8. 输入校验

错误响应包含具体字段，例如：

```json
{"error":{"field":"satellites[2].sigma","message":"pseudorange error standard deviation must be positive and finite, got 0"}}
```

批量字段会继续前缀到出错历元：`epochs[7].satellites[2].sigma`。

拦截情况包括：

- 可见星少于 4；
- 同一历元 PRN 重复；
- 坐标、概略位置或伪距为 NaN/无穷；
- 伪距非正；
- `sigma <= 0`；
- 概略位置距地心小于 1 km；
- 星地距离非法；
- 视线几何矩阵奇异或近奇异（例如所有星共面）；
- 时间戳非正、重复、倒退；
- 批量超过 3600 历元。

## 9. 验收关系与测试

`go test ./...` 覆盖：

- 无故障固定种子蒙特卡洛：SSE 样本均值相对自由度偏差不超过 5%；
- 大偏差注入：检出且排除的恰好是故障星，剔后位置回到真值附近；
- 偏差逐步增大：全星座 SSE 单调增大；
- 所有伪距同加常数：位置不变，仅钟差改变；
- HDOP 越大，HPL 越大；
- Pfa 调小，卡方阈值变大；
- 4 星只定位并标记完好性不可用，5 星可检测，6 星才可排除；
- 故障撤去后必须满足配置恢复历元数，隔离期间不会重新拉回；
- 批量与逐历元结果逐项 `reflect.DeepEqual`；
- 重启后续跑与一口气跑完状态深度相等；
- 重复/倒退时间戳、字段级错误和 HTTP 状态码。

## 10. 构建与运行

```bash
docker build -t gnss-integrity:dev .
docker run --rm -p 8080:8080 \
  -e INTEGRITY_ADDR=:8080 \
  -e INTEGRITY_DATA_DIR=/data \
  -v "$(pwd)/data:/data" \
  gnss-integrity:dev
```

本地运行：

```bash
go test ./...
go run ./cmd/integrity-server
```

默认监听 `:8080`，数据目录默认 `/data`，可通过 `INTEGRITY_DATA_DIR` 和 `INTEGRITY_ADDR` 覆盖。
