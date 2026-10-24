# 「无法解锁 / 无法起飞」问题报告

本文记录一次针对「启动后飞控不解锁、飞机不起飞」的排查：现象、实验、根因和修复。
结论先行：

1. **飞控的解锁标志会被 RC 链路抖动锁死**，而原来的程序一直把 ARM 开关保持在高位，
   没有任何一次「低 → 高」的边沿，所以永远解不开。这是可稳定复现的、真实的程序缺陷。
2. **RC 发送线程会被 MSP 握手阻塞**，握手最长约 1 s，期间 UDP 遥控帧降到 ~1 Hz，
   飞控直接报 `RXLOSS`——这正是触发第 1 条的典型条件。
3. 用户日志里「TCP 已连接、飞控一个字节都不回」的状态在本地**无法复现**（见实验 6），
   但现在会被显式检测、断开并重连，不再静默挂死。

---

## 1. 现象

两类现象，之前混在一起，容易误判：

**A. 静默链路**（用户提供的日志）

```
[msp_node] connected to SITL MSP at 127.0.0.1:5761, RC -> 127.0.0.1:9004
[msp_node] telemetry read failed: MSP timeout waiting for command 108   (每 ~6 s)
[control_node] state telemetry stale (1000000000.00s), commanding idle  (每 2 s)
[control_node] requested ARM switch high                                (每 5 s)
```

TCP 连接建立后再也没有任何回包，`MSP_ATTITUDE`(108)/`MSP_STATUS` 全部超时，
connect 不会失败也不会重连，于是控制节点永远拿不到姿态、永远不解锁。
与此同时 Gazebo 真值位置还在 60 Hz 正常发布（`local position available`），
所以「位置有了但飞不起来」并不矛盾——两条链路本来就是独立的。

**B. 解锁卡死**（本地稳定复现）

MSP 正常、位置正常、`set_arm(true)` 也下发了，但飞控一直报：

```
[SITL] Arming disabled: RXLOSS NOT_DISARMED ARM_SWITCH
[flyrat_msp] arming blocked: flags=0x00030400
```

并且 aircraft 一直不动，直到重启整个仿真栈。

---

## 2. 关键代码事实（Betaflight 侧）

`src/main/fc/core.c`（`processArmingStates`）：

```c
if (!isUsingSticksForArming()) {
    static bool hadRx = false;
    const bool haveRx = isRxReceivingSignal();
    const bool justGotRxBack = !hadRx && haveRx;
    if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
        setArmingDisabled(ARMING_DISABLED_NOT_DISARMED);      // 链路刚恢复 + 开关已在高位
    } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
        unsetArmingDisabled(ARMING_DISABLED_NOT_DISARMED);    // 只有开关拉低才清除
    }
    hadRx = haveRx;
}
...
if (isArmingDisabled() && IS_RC_MODE_ACTIVE(BOXARM)) {
    setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);            // 任何其它阻止项 + 开关高位
} else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
    unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);          // 同样只有拉低才清除
}
```

也就是说：**一旦在「链路抖动」或「其它解锁条件未满足」时开关处于高位，这两个标志会
一直锁存；清除它们的唯一途径是让开关在链路正常时回到低位，再重新拉高。**

原来的 `msp_node` 恰好做不到这一点：`/flyrat/set_arm` 只把 `armed_` 置真，
`build_channels()` 从此每 20 ms 都发 AUX1=1800，控制节点每 5 s 的「重试」也只是
重复设置同一个已为真的标志，开关序列里根本没有边沿。

---

## 3. 实验

全部在 Docker 容器内（ROS 2 Jazzy + Gazebo Harmonic），脚本见 `/tmp`（msp_tcp_probe.py、
arm_latch2.sh 等）。MSP 是 **TCP 5761**（不是 UDP）。

| # | 实验 | 结果 |
|---|---|---|
| 1 | 连续 20 次「连接-发 MSP_API_VERSION-关闭」 | 20/20 应答，10–20 ms |
| 2 | 带 `SO_LINGER 0`（RST）断开后重连 | 正常应答 |
| 3 | 只发 `#` 让飞控进 CLI，再发 MSP 请求 | MSP **仍然应答**（不是静默原因） |
| 4 | 杀掉 Gazebo（`gz sim -s`），保持 SITL | MSP **仍然应答**（与 FDM/物理无关） |
| 5 | 400 个请求不读 + RST 关闭，再连 | 新客户端先收到 ~1400 B 残留应答，随后正常 |
| 6 | 复现用户日志的「连接成功但永不回包」 | **未能复现**；各种断连/重连/CLI/灌包场景飞控都继续服务 |

> 注：早期一版测试脚本漏发了 MSP 头 `$M<`，导致「第一次之后全部静默」的假阳性，
> 该结论已作废（实验 1/2 是修正后的结果）。

**解锁 A/B 实验**（`/tmp/arm_latch2.sh`，地面、不自动起飞）：

1. `set_arm(true)` → `/flyrat/armed = True`
2. `set_arm(false)` → `False`
3. `set_arm(true)` 后在 150 ms 内 `SIGSTOP` `msp_node` 2 s 再 `SIGCONT`
   （模拟 RC 链路中断后恢复，此时飞控看到的开关仍在高位）

| 步骤 | 旧代码 | 新代码 |
|---|---|---|
| 1 首次解锁 | True | True |
| 2 上锁 | False | False |
| 3 链路抖动后重新解锁 | **False（永久锁死）** | **True（≈3 s 内恢复）** |

旧代码在第 3 步后飞控一直打印 `Arming disabled: NOT_DISARMED ARM_SWITCH`，
必须重启仿真栈才能再飞；新代码在 3 s 内自动恢复。

---

## 4. 根因

### R1（主因，可复现）解锁标志锁存 + 开关没有边沿

- 触发条件：任何一次 RC 链路中断（`RXLOSS`）后恢复，而 AUX1 此时为高位。
- 触发源在本项目里很常见：MSP 重连握手、节点重启、CPU 抖动、上一轮进程退出留下的抖动。
- 结果：`NOT_DISARMED` 与 `ARM_SWITCH` 同时置位，而程序只发高位，标志永远不会清除。
- 用户看到的 `flags=0x00030400` 是同一状态的另一面：只要还有别的阻止项
  （MSP override / prearm / 校准等）在开关高位时成立，`ARM_SWITCH` 就会一直挂着。

### R2（放大器）RC 发送线程被 MSP 握手阻塞

`sender_loop()` 每个周期都调用 `connect_links()`。链接断开时这个函数会做一次
「连接 + 等 MSP_API_VERSION」的握手，超时 0.3 s、失败后退避 1 s，而且整个握手
持有 `link_mutex_`。于是：

- 握手期间 50 Hz 的 UDP 遥控帧（9004）被拖到 ~1 Hz；
- 飞控判定 `RXLOSS`；
- 链路恢复的瞬间开关仍在高位 → 正好命中 R1。

两个问题互相喂养，这就是「有时能飞、有时死活不解锁」的概率性来源。

### R3（用户日志）飞控不处理 MSP

「TCP 已接受但永不回包」在本地用尽手段未能复现，可能来自用户机器上的残留进程/端口
归属，或飞控侧一次性的状态异常。无论成因，原来的程序对这种状态的**反应**是错误的：
它把「TCP 连接成功」当成「链路可用」，于是一直挂在一个哑连接上，既不解锁也不报错。

---

## 5. 修复

`flyrat_play/ros2_ws/src/flyrat_msp/src/msp_node.cpp`

1. **解锁加边沿**：`/flyrat/set_arm(true)` 先让 AUX1 保持低 250 ms 再拉高；
   在「请求解锁但飞控未 armed」期间每 3 s 自动重复一次脉冲。这样任何一次锁存
   都会在几秒内被一次完整的「低 → 高」清除。
2. **发送线程不再做握手**：`sender_loop()` 只发 UDP 遥控帧和（已连接时的）
   `MSP_SET_RAW_RC`；接收机在 `start()` 打开、失败由遥测线程补开。
   链路的建立与校验只发生在遥测线程。
3. **链路校验**：`connect()` 之后必须收到一次 `MSP_API_VERSION` 才算连上；
   失败则关闭、退避 1 s，并打印

   ```
   SITL at 127.0.0.1:5761 accepted the TCP connection but never answered
   MSP_API_VERSION (...). The flight controller is not processing MSP: restart
   the simulation stack (a stale SITL or another program may own port 5761).
   ```

4. **静默链路自愈**：连续 3 个遥测周期失败就主动断开，下一周期重连并重新校验。
5. `MspClient::close()` 在 `send_mutex_` 保护下关闭 fd，避免 RC 线程写进被复用的描述符。
6. `control_node` 的 `state telemetry stale` 提示改为指向 MSP 链路日志，而不是只说
   「commanding idle」。

---

## 6. 验证

- 修复后正常起飞：`arming blocked` 只在启动最初几秒出现 3 次（飞控自检），
  随后 `ARM switch -> high` → `mpc:` 日志开始 → 悬停/轨迹正常。
- A/B 实验（见 §3）：旧代码链路抖动后永久锁死，新代码 3 s 内恢复。
- 静默链路：用占位客户端占住 TCP 槽位启动整套栈，节点每 2 s 打印上述明确错误，
  从不误报「已连接」；释放槽位后 10 ms 内自动接上并恢复飞行。
- `libflyrat` 单元测试 3/3，`colcon build` 两个包通过。

---

## 7. 现场怎么判断

| 日志 | 含义 | 处理 |
|---|---|---|
| `accepted the TCP connection but never answered MSP_API_VERSION` | 端口被别的进程/僵尸 SITL 占着，或飞控没在跑 MSP | 重启仿真栈；确认 `ss -ltnp \| grep 5761` 只有 `betaflight_SITL` |
| `MSP link went silent (...); dropped it` | 链路中途失联，已自动重连 | 观察后续是否出现 `connected to SITL MSP` |
| `arming blocked: flags=0x...` 持续超过 ~10 s | 有其它解锁阻止项（MSP override、prearm、校准、油门） | 看 SITL 自己的 `Arming disabled: ...` 行 |
| `Arming disabled: ... NOT_DISARMED ARM_SWITCH` | 开关锁存（本文 R1） | 已被 3 s 自动脉冲覆盖；若仍出现说明 RC 根本没恢复 |

相关启动参数：`msp_override:=false` 可以避开 `ARMING_DISABLED_MSP`，它在
`betaflight_sim.txt` 里对应 AUX2 的 MSP OVERRIDE box。
