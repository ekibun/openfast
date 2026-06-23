# StC (Structural Control) Simulink Interface

## 概述

本次修改在 OpenFAST 的 Simulink 接口中新增了 **150 个 StC 控制信号输入**，使得用户可以通过 Simulink 直接控制结构控制（Structural Control）设备，而不再仅限于 Bladed DLL 接口。

## InputAry 布局

修改后 `NumFixedInputs` 从 51 扩展为 **201**。Signal 输入数组 `InputAry` 的完整布局如下：

| 位置 | 名称 | 说明 | 单位 |
|------|------|------|------|
| 1 | GenTrq | 发电机扭矩 | N·m |
| 2 | ElecPwr | 电功率 | W |
| 3 | YawPosCom | 偏航位置指令 | rad |
| 4 | YawRateCom | 偏航速率指令 | rad/s |
| 5–7 | BlPitchCom(1:3) | 叶片 1–3 桨距角指令 | rad |
| 8 | HSSBrFrac | 高速轴制动力比例 | - |
| 9–11 | BlAirfoilCom(1:3) | 叶片 1–3 翼型控制 | - |
| 12–31 | CableDeltaL(1:20) | 缆绳控制长度增量 通道 1–20 | m |
| 32–51 | CableDeltaLdot(1:20) | 缆绳控制长度变化率 通道 1–20 | m/s |
| **52–81** | **StCCmdStiff** | **StC 刚度指令 (3 DOF × 10 通道)** | **N/m** |
| **82–111** | **StCCmdDamp** | **StC 阻尼指令 (3 DOF × 10 通道)** | **N/(m/s)** |
| **112–141** | **StCCmdBrake** | **StC 制动力指令 (3 DOF × 10 通道)** | **N** |
| **142–171** | **StCCmdForce** | **StC 主动力指令 (3 DOF × 10 通道)** | **N** |
| **172–201** | **StCCmdMoment** | **StC 主动力矩指令 (3 DOF × 10 通道)** | **N·m** |

## StC 信号详解

每个 StC 信号类型包含 **30 个值**（3 DOF × 10 个控制通道），组织方式为：

```
对于通道 i (i = 1, 2, ..., 10)：
  值 (i-1)*3 + 1  →  X 方向 (DOF 1)
  值 (i-1)*3 + 2  →  Y 方向 (DOF 2)
  值 (i-1)*3 + 3  →  Z 方向 (DOF 3)
```

### 1. StCCmdStiff — 刚度指令（位置 52–81）

控制 StC 设备的**刚度**（Stiffness）。正值增加刚度，使设备"更硬"；负值可能减小刚度（取决于具体实现）。

- 通道 1, DOF X (pos 52), DOF Y (pos 53), DOF Z (pos 54)
- 通道 2, DOF X (pos 55), DOF Y (pos 56), DOF Z (pos 57)
- ...
- 通道 10, DOF X (pos 79), DOF Y (pos 80), DOF Z (pos 81)

### 2. StCCmdDamp — 阻尼指令（位置 82–111）

控制 StC 设备的**阻尼**（Damping）。正值增加阻尼，消耗更多能量。

- 通道 1, DOF X (pos 82), DOF Y (pos 83), DOF Z (pos 84)
- ...
- 通道 10, DOF X (pos 109), DOF Y (pos 110), DOF Z (pos 111)

### 3. StCCmdBrake — 制动力指令（位置 112–141）

控制 StC 设备的**制动力**（Brake force）。用于锁定或限制设备运动范围。

- 通道 1, DOF X (pos 112), DOF Y (pos 113), DOF Z (pos 114)
- ...
- 通道 10, DOF X (pos 139), DOF Y (pos 140), DOF Z (pos 141)

### 4. StCCmdForce — 主动力指令（位置 142–171）

施加于 StC 设备的**主动控制力**（Active control force）。直接对质量块施加力。

- 通道 1, DOF X (pos 142), DOF Y (pos 143), DOF Z (pos 144)
- ...
- 通道 10, DOF X (pos 169), DOF Y (pos 170), DOF Z (pos 171)

### 5. StCCmdMoment — 主动力矩指令（位置 172–201）

施加于 StC 设备的**主动控制力矩**（Active control moment）。

- 通道 1, DOF X (pos 172), DOF Y (pos 173), DOF Z (pos 174)
- ...
- 通道 10, DOF X (pos 199), DOF Y (pos 200), DOF Z (pos 201)

## 控制通道与 StC 实例的对应关系

控制通道（1–10）在 StC 输入文件中通过 `StC_CChan` 参数分配给具体的 StC 实例。例如：

```
Blade TMD 1:  StC_CChan = 1   →  使用通道 1 的指令
Nacelle TMD:  StC_CChan = 3   →  使用通道 3 的指令
Tower TMD:    StC_CChan = 5   →  使用通道 5 的指令
```

多个 StC 实例可以共用同一个通道（此时它们的测量值会被平均后反馈给控制器）。

## 使用方法

### 1. StC 输入文件配置

在 StC 输入文件中设置 `StC_CMODE = 4`（Simulink 外部控制）：

```
StC_CMODE   4          ! 4 = Active control via Simulink (EXTERN)
StC_CChan   1          ! 使用控制通道 1
```

### 2. Simulink 配置

Simulink 模型中的 InputAry 需要扩展为 **201 个元素**（之前为 51）。未使用的 StC 信号位置填 0 即可。

### 3. 信号流

```
Simulink InputAry(52:201)
  → FAST_SetExternalInputs (FAST_Library.f90)
  → m_FAST%ExternInput%StCCmdStiff/Damp/Brake/Force/Moment
  → SrvD_SetExternalInputs (FAST_Subs.f90)
  → u_SrvD%ExternalStCCmdStiff/Damp/Brake/Force/Moment
  → StCControl_CalcOutput (ServoDyn.f90, ControlMode_EXTERN 分支)
  → StC_CmdStiff/Damp/Brake/Force/Moment
  → SetStCInput_CtrlChans → u_StC%CmdStiff/Damp/Brake/Force/Moment
  → StC_ActiveCtrl_StiffDamp (StrucCtrl.f90)
  → 结构控制物理计算
```

## 相关文件修改

| 文件 | 修改内容 |
|------|----------|
| `FAST_Library.h` | 新增 `MAXIMUM_STC_CONTROL=10`，`NumFixedInputs` 51→201 |
| `FAST_Library.f90` | `FAST_SetExternalInputs` 新增 52–201 的数据提取 |
| `FAST_Types.f90` | `FAST_ExternInputType` 新增 5 个 StC 数组 (各 30 元素) |
| `FAST_Subs.f90` | `SrvD_SetExternalInputs` 新增 flat→2D 转换 |
| `ServoDyn_Types.f90` | `SrvD_InputType` 新增 5 个 ExternalStC 2D 数组 |
| `ServoDyn.f90` | 初始化分配、`StCControl_CalcOutput` EXTERN 分支 |
| `StrucCtrl.f90` | 启用 `CMODE_ActiveEXTERN=4` |
| `ServoDyn_IO.f90` | `WrSumInfo4Simulink` 更新文档 |

## 常量定义

```c
#define MAXIMUM_STC_CONTROL 10           // 最大 StC 控制通道数
#define MAXIMUM_STC_STIFF  (3 * 10) = 30 // 刚度值数量
#define MAXIMUM_STC_DAMP   (3 * 10) = 30 // 阻尼值数量
#define MAXIMUM_STC_BRAKE  (3 * 10) = 30 // 制动力值数量
#define MAXIMUM_STC_FORCE  (3 * 10) = 30 // 主动力值数量
#define MAXIMUM_STC_MOMENT (3 * 10) = 30 // 主动力矩值数量
// NumFixedInputs = 51 + 150 = 201
```
