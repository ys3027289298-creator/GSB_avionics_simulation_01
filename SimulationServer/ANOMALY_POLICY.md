# 点火状态机异常处理策略

`ignition_fsm.py` 把顶点检测与点火触发建模为带幂等约束的线性状态机：

```
IDLE -> APOGEE_DETECTED -> PYRO1_FIRED -> PYRO2_FIRED
```

另有一个与主状态正交的 **降级（degraded）标志**：进入降级后状态机继续
推进，但会话被标记为不可信，测试报告中会列出降级原因。

状态机是事件流的纯函数（无 I/O、无随机性），同一事件序列重放任意多次
结果完全一致。遥测噪声的确定性由 `server.py` 的 `--seed` /
`sim.seed` 配置保证。

## 处理分类

### 拒绝（REJECTED）——永不采纳，状态不变

| 异常 | 判定 |
| --- | --- |
| 高度越界 | `alt < 0`、`alt > max_altitude_m`（默认取真实顶点高度的 1.5 倍）、`NaN`、`Inf` |
| 时间戳倒退 | 事件时间早于上一个已接受事件的时间 |
| 顺序违规 | PYRO2 先于 PYRO1 到达 |
| 间隔不足 | PYRO2 与 PYRO1 间隔小于 `min_pyro_separation_s` |
| 通道未使能 | 配置关闭 PYRO2 时收到 PYRO2 事件 |

被拒绝的事件不写入 `session["events"]`，只记入 `session["anomalies"]`；
之后的合法事件仍可被接受（例如过近的 PYRO2 被拒后，晚些时候的合法
PYRO2 依然生效）。

### 幂等忽略（DUPLICATE）——不改变状态的重复

| 异常 | 判定 |
| --- | --- |
| 相同时间戳 | `(type, sim_time)` 幂等键已存在（时间戳按毫秒量化，与服务器 `round(t, 3)` 一致） |
| 重复点火 / 已点过火 | 该通道已有已接受事件，**首次点火永远胜出**，后续一律忽略 |

这保证单次点火不变量：每个通道在状态机中至多一条已接受记录，
重复运行同一事件序列不会累积出第二次点火。

### 重试（RETRY）——等待重传，状态不变

| 异常 | 判定 |
| --- | --- |
| 数据缺失 | 字段不足、事件类型为空、高度字段为空 |
| 无法解析 | 高度非数值、未知事件类型 |

重试不计入幂等键、不影响时间戳单调性；飞控重传合法事件后正常接受。
连续解析失败达到 `max_consecutive_parse_errors`（默认 5）时进入降级。

### 降级（DEGRADED）——继续运行但标记不可信

| 异常 | 判定 |
| --- | --- |
| 传感器值抖动 | 同一通道重复上报的高度差超过 `jitter_threshold_m`（默认 25 m） |
| 顶点报告丢失 | 收到 PYRO1 但此前没有 APOGEE 事件 |
| 重试风暴 | 连续缺失/无法解析事件超过阈值 |

## 可调参数

均来自 `config.json` 的 `validation` 段（缺省值见括号）：

- `min_pyro_separation_s`（2.0）
- `jitter_threshold_m`（25.0）
- `max_consecutive_parse_errors`（5）
- 高度上界由飞行仿真自动推导：`(apogee - elevation) * 1.5`

## 复现与回归

- `python3 reproduce_anomalies.py` —— 用构造事件流逐类复现上述异常，
  不依赖 rocketpy、串口、socket 或遥测服务器。
- `python3 -m unittest test_ignition_fsm -v` —— 回归测试，包含带种子的
  伪随机异常事件流，验证同一序列多次运行结果一致。
