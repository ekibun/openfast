# ServoDyn StC/TMD 安装截面运动输出修改说明

本文档说明本次对 ServoDyn StC/TMD 输出的修改，并解释为什么最终采用与 Simulink StC 控制输入相似的“控制通道”规则，而不是按 TMD 安装位置类型排列输出通道。

修改涉及：

- `modules/servodyn/src/ServoDyn.f90`
- `modules/servodyn/src/ServoDyn_IO.f90`

## 1. 最近 3 个 commit 的相关结论

最近 3 个 commit 为：

1. `d143a0c`：启用 `CMODE_ActiveEXTERN` 并支持局部坐标 prescribed force。
2. `19a2ede`：修正 StC DLL routines 在 EXTERN 兼容性下的 guard。
3. `49967a2`：新增 StC Simulink interface。

其中与本次输出通道排列最相关的是 `49967a2`。

该 commit 将 Simulink `InputAry` 从 51 扩展到 201，并新增 150 个 StC 控制输入：

| 范围 | 信号 | 布局 |
| --- | --- | --- |
| 52-81 | `StCCmdStiff` | 3 DOF x 10 控制通道 |
| 82-111 | `StCCmdDamp` | 3 DOF x 10 控制通道 |
| 112-141 | `StCCmdBrake` | 3 DOF x 10 控制通道 |
| 142-171 | `StCCmdForce` | 3 DOF x 10 控制通道 |
| 172-201 | `StCCmdMoment` | 3 DOF x 10 控制通道 |

参考代码：

- `modules/openfast-library/src/FAST_Library.f90`
- `modules/openfast-library/src/FAST_Subs.f90`
- `modules/servodyn/src/ServoDyn.f90`
- `docs/StC_Simulink_Interface.md`

## 2. 为什么 Simulink 荷载输入不区分安装位置

Simulink StC 输入没有按安装位置分成 `BStC/NStC/TStC/SStC`，而是统一按 `StC_CChan` 控制通道组织。

核心原因是：Simulink 外部接口需要固定长度、固定语义的数组。如果按安装位置排列，接口会随着 blade/nacelle/tower/substructure 的实例数量、blade 数量和安装位置组合膨胀，接口会变得很难维护。

因此 commit `49967a2` 采用了通道式规则：

```text
通道 i 的 X/Y/Z 三个 DOF：
  flat index = (i-1)*3 + 1 : (i-1)*3 + 3
```

`FAST_Subs.f90` 中把 Simulink flat array 转成 ServoDyn 的二维数组：

```fortran
u_SrvD%ExternalStCCmdForce(1:3,i) = &
   m_FAST%ExternInput%StCCmdForce((i-1)*3+1:(i-1)*3+3)
```

之后 `ServoDyn.f90` 中直接把这些通道值送到 StC 控制矩阵：

```fortran
StC_CmdForce(1:3,1:p%NumStC_Control) = &
   u%ExternalStCCmdForce(1:3,1:p%NumStC_Control)
```

具体哪个 TMD 使用哪个通道，不由 Simulink 接口本身决定，而由每个 StC 输入文件中的 `StC_CChan` 决定。例如：

```text
Tower TMD:   StC_CChan = 1
Nacelle TMD: StC_CChan = 2
Blade TMD:   StC_CChan = 3
```

这样 Simulink 只需要知道“控制通道 1/2/3”，不需要知道设备安装在 tower、nacelle 还是 blade。

## 3. 为什么输出也改成相同规则

最初实现曾按安装位置生成输出名，例如：

```text
TStC1_PX
NStC1_PX
BStC1_B2_PX
SStC1_PX
```

这种规则直接、清楚，但会带来两个问题：

1. 通道数量膨胀：N/T/S 各 4 个实例，BStC 还要乘以 blade 数，18 个运动量展开后约 504 个新增通道。
2. 与 Simulink StC 控制输入规则不一致：外部控制按 `StC_CChan` 管理，而输出按安装位置管理，使用时需要用户自己再次建立映射。

现在改为与 Simulink 一致的通道式输出：

```text
StCCh1_PX
StCCh1_VX
StCCh1_AX
...
StCCh10_ALZ
```

这样输出端也只关心控制通道号。安装位置仍然由 `StC_CChan` 在 StC 输入文件中绑定。

## 4. 新增输出通道

每个 `StCCh1` 到 `StCCh10` 有 18 个输出量：

| 后缀 | 含义 | 单位 |
| --- | --- | --- |
| `PX/PY/PZ` | 安装截面全局位置 | `m` |
| `VX/VY/VZ` | 安装截面全局平动速度 | `m/s` |
| `AX/AY/AZ` | 安装截面全局平动加速度 | `m/s^2` |
| `RX/RY/RZ` | 安装截面全局 ZYX Euler 方位角 | `deg` |
| `WX/WY/WZ` | 安装截面全局角速度 | `deg/s` |
| `ALX/ALY/ALZ` | 安装截面全局角加速度 | `deg/s^2` |

示例：

```text
StCCh1_PX
StCCh1_PY
StCCh1_PZ
StCCh1_VX
StCCh1_AX
StCCh1_RX
StCCh1_WX
StCCh1_ALX

StCCh10_PX
StCCh10_WZ
StCCh10_ALZ
```

输出名在 `SetOutParam` 中会被转成大写，因此大小写不敏感。

## 5. TMD 安装截面运动量来自哪里

StC/TMD 安装位置由 StC 输入文件中的：

```text
StC_P_X
StC_P_Y
StC_P_Z
```

定义。`StrucCtrl.f90` 初始化时将其写入 `InitOut%RelPosition`，并通过 `MeshCreate` 创建 StC input motion mesh。

该 mesh 启用了：

- `TranslationDisp`
- `Orientation`
- `TranslationVel`
- `RotationVel`
- `TranslationAcc`
- `RotationAcc`

运行时 ServoDyn 会把来自结构模块的 StC motion mesh transfer 到：

```fortran
m%u_BStC(1,i)%Mesh(j)
m%u_NStC(1,i)%Mesh(1)
m%u_TStC(1,i)%Mesh(1)
m%u_SStC(1,i)%Mesh(1)
```

因此安装截面运动输出直接从这些 input mesh 中读取。

## 6. 代码修改一：输出索引范围

原有 ServoDyn 输出最大索引为 520。本次新增：

```fortran
INTEGER(IntKi), PARAMETER :: MaxStCControlOuts = 10
INTEGER(IntKi), PARAMETER :: StC_Kin_Num       = 18
INTEGER(IntKi), PARAMETER :: StC_Kin_Start     = 521
INTEGER(IntKi), PARAMETER :: MaxOutPts = StC_Kin_Start + MaxStCControlOuts*StC_Kin_Num - 1
```

因此新增输出范围为：

```text
521-700
```

布局为：

```text
StCCh1  : 521-538
StCCh2  : 539-556
...
StCCh10 : 683-700
```

参考代码：

- `modules/servodyn/src/ServoDyn_IO.f90`

## 7. 代码修改二：OutList 动态解析

ServoDyn 原有输出表由 `ValidParamAry`、`ParamIndxAry`、`ParamUnitsAry` 三个静态数组维护。由于新增通道按规则生成，手工塞入静态表并保持排序没有必要。

本次保留原有静态表逻辑；当静态表找不到输出名时，才调用：

```fortran
Parse_StC_Kin_OutParam(...)
```

该函数只识别：

```text
StCCh1_<suffix> ... StCCh10_<suffix>
```

索引计算为：

```fortran
StC_Kin_Index_Ch = StC_Kin_Start + (chan-1)*StC_Kin_Num + kin - 1
```

无效条件为：

```fortran
p%NumStC_Control < chan
```

也就是说，如果输入文件中最大 `StC_CChan` 只有 3，那么 `StCCh4_PX` 会被标记为无效输出。

## 8. 代码修改三：按 StC_CChan 聚合输出

`SrvD_CalcOutput` 中原有 StC 输出仍然负责已有的 `BStC/TStC/NStC/SStC` 相对位移和载荷输出：

```fortran
call Set_BStC_Outs(...)
call Set_NStC_Outs(...)
call Set_TStC_Outs(...)
call Set_SStC_Outs(...)
```

然后新增统一调用：

```fortran
call Set_StC_Kin_Ch_Outs( p, m%u_BStC, m%u_NStC, m%u_TStC, m%u_SStC, AllOuts )
```

这个函数扫描所有 StC：

- `BStC(i)` 的每个 blade point 使用 `p%BStC(i)%StC_CChan(j)`
- `NStC(i)` 使用 `p%NStC(i)%StC_CChan(1)`
- `TStC(i)` 使用 `p%TStC(i)%StC_CChan(1)`
- `SStC(i)` 使用 `p%SStC(i)%StC_CChan(1)`

若 `StC_CChan` 为 0，则不输出到任何 `StCCh` 通道。

## 9. 多个 StC 共用一个通道时如何处理

Simulink/DLL 测量值已有类似逻辑：多个 StC 实例共用一个 `StC_CChan` 时，会对测量量求平均。

本次输出也沿用同样规则：

1. 对同一个 `StCChX` 的所有安装截面运动量先累加。
2. 记录该通道累加了多少个 StC mesh point。
3. 如果数量大于 1，则除以数量，输出平均值。

这使输出含义与控制通道语义一致：`StCChX_*` 表示通道 X 对应的安装截面运动量；如果通道 X 服务多个设备，则输出这些设备的平均安装截面运动量。

## 10. 每个运动量如何计算

位置：

```fortran
PX = u%Mesh(j)%Position(1,1) + u%Mesh(j)%TranslationDisp(1,1)
PY = u%Mesh(j)%Position(2,1) + u%Mesh(j)%TranslationDisp(2,1)
PZ = u%Mesh(j)%Position(3,1) + u%Mesh(j)%TranslationDisp(3,1)
```

速度：

```fortran
VX/VY/VZ = u%Mesh(j)%TranslationVel(:,1)
```

加速度：

```fortran
AX/AY/AZ = u%Mesh(j)%TranslationAcc(:,1)
```

方位角：

```fortran
RotDisp = EulerExtractZYX(u%Mesh(j)%Orientation(:,:,1))
RX/RY/RZ = RotDisp * R2D
```

角速度：

```fortran
WX/WY/WZ = u%Mesh(j)%RotationVel(:,1) * R2D
```

角加速度：

```fortran
ALX/ALY/ALZ = u%Mesh(j)%RotationAcc(:,1) * R2D
```

## 11. 使用示例

如果 tower TMD 的 StC 输入文件中设置：

```text
StC_CChan 1
```

则可在 ServoDyn `OutList` 中写：

```text
StCCh1_PX
StCCh1_PY
StCCh1_PZ
StCCh1_VX
StCCh1_AX
StCCh1_RX
StCCh1_WX
StCCh1_ALX
```

如果 nacelle TMD 设置：

```text
StC_CChan 2
```

则输出：

```text
StCCh2_PX
StCCh2_VX
StCCh2_WZ
```

## 12. 注意事项

1. 新输出依赖 `StC_CChan`。如果某个 StC 的 `StC_CChan=0`，它不会出现在 `StCCh*` 输出中。
2. 若不使用 EXTERN/DLL 控制，但仍希望输出通道式安装截面运动，需要确保 StC 初始化后 `p%StC_CChan` 没有被清零；当前 StrucCtrl 逻辑通常只在 active DLL/EXTERN 模式保留控制通道。
3. 多个 StC 共用一个通道时输出平均值，和现有测量信号的通道语义保持一致。
4. `RX/RY/RZ` 使用 `EulerExtractZYX`，与 OpenFAST 其他从 mesh orientation 提取欧拉角的位置保持风格一致。

## 13. 验证

已运行：

```bash
git diff --check
```

用于检查 trailing whitespace 和 patch 格式。

当前环境没有可用 Fortran 编译器，因此无法完成实际 `servodynlib` 编译验证。此前 CMake 报错为：

```text
No CMAKE_Fortran_COMPILER could be found.
```
