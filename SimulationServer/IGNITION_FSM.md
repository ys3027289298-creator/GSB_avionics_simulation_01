# 点火状态机（Ignition FSM）

`SimulationServer` 持续接收遥测帧并回收固件回传的 `EVENT,<TYPE>,<alt>` 事件
（`TYPE ∈ APOGEE / PYRO1 / PYRO2`）。`ignition_fsm.py` 把整个点火过程建模为
带幂等约束的状态机，由 `server.py` 的 `run_session()` 调用。**线协议与消息字段名
保持不变**：下行仍是 10 字段 CSV 遥测帧，上行仍是 `EVENT,类型,高度`。

## 状态

```
WAIT_APOGEE --APOGEE--> APOGEE_CONFIRMED --PYRO1--> PYRO1_FIRED --PYRO2--> PYRO2_FIRED
```

- `pyro2: false`（单伞模式）时，`PYRO1_FIRED` 为终态，收到 `PYRO2` 直接降级。
- 每种事件只“提交”一次；已提交值（首个合法事件）作为评分基准，永不被覆盖。

## 幂等约束（两层去重键）

1. **精确重传**：键为 `(事件类型, 时间戳 ms 取整, 高度 cm 取整)`。相同键恒返回
   `DUPLICATE`，即使晚到也不影响状态（重放安全）。
2. **抖动/重试窗口**：同类型事件若在已提交时间的 `dedup_window_s`（默认 0.5s）
   之内、高度差在 `dedup_alt_tolerance_m`（默认 5m）之内，视为重试，返回
   `DUPLICATE`；超出窗口且参数冲突则判定为**重复点火**。

相同时间戳上的不同事件按到达顺序依次处理（APOGEE 与 PYRO1 可在同一 tick 提交）。

## 异常处置表

| 异常 | 处置 | kind | 说明 |
|---|---|---|---|
| 字段缺失（`EVENT,PYRO1`、`EVENT`） | **拒绝 REJECT** | `missing_fields` | 丢弃该行，状态不变，继续监听 |
| 高度非数值（`abc`） | **拒绝 REJECT** | `non_numeric_alt` | 同上，可重试 |
| 高度为 NaN / Inf（数据缺失的数值表现） | **拒绝 REJECT** | `non_finite_alt` | 同上，可重试 |
| 高度越界（`< 0` 或 `> 1.5 × 真实顶点高度`） | **拒绝 REJECT** | `altitude_out_of_bounds` | 物理不可能，丢弃，后续合法事件可重试 |
| 时间戳回退（早于已接受时刻，相同时间戳不算） | **拒绝 REJECT** | `stale_timestamp` | 丢弃，状态不变 |
| 未知事件类型 | **拒绝 REJECT** | `unknown_event` | 协议噪声，不降级 |
| 精确重传（时间戳与高度完全相同） | **重试幂等 DUPLICATE** | `exact_retransmission` | 空操作，不是错误 |
| 传感器抖动导致的成簇上报（窗口内、高度容差内） | **重试幂等 DUPLICATE** | `retry_in_dedup_window` | 首个提交，其余空操作 |
| 遥测帧缺失/断流 | **重试（等待）** | — | 无事件即不触发任何迁移，状态机原地等待下一帧 |
| 已点过火且参数一致（窗口内） | **重试幂等 DUPLICATE** | `retry_in_dedup_window` | 同抖动处理 |
| 重复点火（已提交后，超出窗口/容差的再次 PYRO） | **降级 DEGRADED** | `double_fire` | 不提交新值，首个点火值保留，置降级锁 |
| 乱序（APOGEE 前 PYRO1、PYRO1 前 PYRO2） | **降级 DEGRADED** | `out_of_order` | 事件不提交，继续记录 |
| PYRO2 已禁用却收到点火 | **降级 DEGRADED** | `channel_disabled` | 事件不提交 |
| 序列完成后出现新点火 | **降级 DEGRADED** | `double_fire` | 终态保护 |
| PYRO1→PYRO2 间隔小于 `min_pyro_separation_s` | **接受但降级 ACCEPT+DEGRADED** | `insufficient_separation` | 事件有效但记录降级，顺序校验同时判 FAIL |

三类语义：

- **拒绝（REJECT）**：针对畸形或物理不可能的输入；只丢弃该行，状态机不变并继续
  监听，固件随后用合法事件重试即可成功（拒绝后重试的用例见测试
  `test_retry_after_rejected_frame_still_works`）。
- **重试（DUPLICATE / 等待）**：重传、抖动簇、窗口内重复上报以及断流，全部幂等
  空操作，不记为错误。
- **降级（DEGRADED）**：真实飞控逻辑异常（重复点火、乱序、禁用通道、间隔不足）；
  置会话级 `degraded` 锁，首个提交值保留用于评分，会话继续进行并在报告末尾打印
  `IGNITION FSM ANOMALIES` 明细。

## 确定性保证

- 状态机是事件序列的纯函数；无时钟读取、无随机数、无 I/O。
- `test_same_stream_repeated_runs_identical` 对同一固定种子伪随机流运行 3 次，
  断言决策序列、异常列表、最终状态、三个提交值完全一致。
- `test_pseudo_random_stream_regression` 锁定整条决策序列，防止行为回归。
- 需要遥测噪声本身也可复现时，在 `config.json` 的 `sim.seed` 填一个整数
  （默认 `null`，不播种）。

## 运行

```bash
# 异常复现（仅标准库，无外部服务/硬件）
python3 anomaly_demo.py

# 回归测试
python3 -m unittest test_ignition_fsm -v
```

## 相关文件

- `ignition_fsm.py` — 纯标准库状态机，无 rocketpy/serial/requests 依赖。
- `server.py` — `run_session()` 用 `IgnitionFSM.handle_line()` 统一处理事件；
  会话 JSON 新增 `anomalies` 与 `degraded` 两个字段（原有字段名不变）。
- `test_ignition_fsm.py` — 固定种子伪随机事件流的回归测试。
- `anomaly_demo.py` — 构造事件流复现全部异常类别。
