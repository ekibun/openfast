# StC Simulink 接口修改详解

本文档逐文件、逐段解释每一处修改的内容、作用和参考的现有代码模式。

所有修改遵循 **Cable 控制信号**（`CableDeltaL` / `CableDeltaLdot`）的已有模式。Cable 控制是唯一一个已完整实现从 Simulink → FAST_Library → FAST_Subs → ServoDyn → 子模块 路径的外部控制信号。

---

## 1. FAST_Library.h — C 头文件常量

**文件**: `modules/openfast-library/src/FAST_Library.h`

### 1.1 新增常量 (line 68)

```c
#define MAXIMUM_STC_CONTROL 10
```

**作用**: 定义 Simulink 接口中 StC 控制通道的最大数量。与 `StrucCtrl.f90:2332` 中 DLL 通道上限 `> 10` 检查保持一致。

**参考**: `MAXIMUM_CABLE_DELTAL 20` / `MAXIMUM_CABLE_DELTALDOT 20` (line 66–67)。Cable 有 20 个通道，StC 取 10（StC 通道更复杂，每个通道占 3 DOF × 5 信号类型 = 15 个 InputAry 位置）。

### 1.2 修改 NumFixedInputs (line 73)

```c
// 旧值: (2 + 2 + MAXIMUM_BLADES + 1 + MAXIMUM_AFCTRL + MAXIMUM_CABLE_DELTAL + MAXIMUM_CABLE_DELTALDOT)  // = 51
// 新值: ... + 5 * 3 * MAXIMUM_STC_CONTROL)  // = 201
```

**作用**: InputAry 固定长度从 51 扩展到 201，新增 150 个 StC 输入位。

**参考**: 原有公式结构，直接追加 `5 * 3 * MAXIMUM_STC_CONTROL`。

### 1.3 修改 MAXInitINPUTS (line 71)

```c
#define MAXInitINPUTS 203
```

**作用**: 初始化输入数组的最大大小。`203 = NumFixedInputs(201) + 2`（预留 lidar 等可选参数）。需与 `FAST_Library.f90` 中的值保持同步。

**参考**: 原有值 `53 = 51 + 2`，保持一致的比例关系。

### 1.4 更新固定输入文档 (line 74–88)

**作用**: 在注释中新增 52–201 的 StC 信号说明，便于用户查阅 InputAry 布局。

**参考**: 原有注释中对 position 1–51 的描述格式。

---

## 2. FAST_Library.f90 — Fortran 接口层

**文件**: `modules/openfast-library/src/FAST_Library.f90`

### 2.1 修改常量 (line 40–41)

```fortran
INTEGER(IntKi), PARAMETER :: MAXInitINPUTS = 203
INTEGER(IntKi), PARAMETER :: NumFixedInputs = 201
```

**作用**: 与 C 头文件保持一致。

**参考**: 原有值 53 和 51。

### 2.2 FAST_SetExternalInputs — 新增 StC 数据提取 (line 416–420)

```fortran
m_FAST%ExternInput%StCCmdStiff  = InputAry(52:81)
m_FAST%ExternInput%StCCmdDamp   = InputAry(82:111)
m_FAST%ExternInput%StCCmdBrake  = InputAry(112:141)
m_FAST%ExternInput%StCCmdForce  = InputAry(142:171)
m_FAST%ExternInput%StCCmdMoment = InputAry(172:201)
```

**作用**: 将 Simulink 传入的 InputAry 中 52–201 位置的值赋给 `FAST_ExternInputType` 中对应的 StC 字段。

**参考**: 
```fortran
m_FAST%ExternInput%CableDeltaL    = InputAry(12:31)    ! line 416
m_FAST%ExternInput%CableDeltaLdot = InputAry(32:51)    ! line 417
```
Cable 控制使用相同的切片赋值模式（连续区间直接赋值）。

---

## 3. FAST_Types.f90 — FAST_ExternInputType 类型定义

**文件**: `modules/openfast-library/src/FAST_Types.f90`

### 3.1 新增 5 个 StC 数组 (line 562–566)

```fortran
REAL(ReKi), DIMENSION(1:30) :: StCCmdStiff = 0.0_ReKi
REAL(ReKi), DIMENSION(1:30) :: StCCmdDamp  = 0.0_ReKi
REAL(ReKi), DIMENSION(1:30) :: StCCmdBrake = 0.0_ReKi
REAL(ReKi), DIMENSION(1:30) :: StCCmdForce = 0.0_ReKi
REAL(ReKi), DIMENSION(1:30) :: StCCmdMoment = 0.0_ReKi
```

**作用**: 在 `FAST_ExternInputType` 中存储从 Simulink 接收的 StC 控制值。每个数组大小为 30（3 DOF × 10 通道），与 InputAry 中的连续区间一一对应。

为何使用 **flat 1D 数组** 而非 2D `(3,10)`？
- InputAry 本身是 1D 数组，flat 结构可以直接用切片赋值 `InputAry(52:81)`，无需循环。
- 后续在 `FAST_Subs.f90` 中再转换为 2D `(3, NumStC_Control)` 传给 ServoDyn。

**参考**:
```fortran
REAL(ReKi), DIMENSION(1:20) :: CableDeltaL    = 0.0_ReKi   ! line 560
REAL(ReKi), DIMENSION(1:20) :: CableDeltaLdot = 0.0_ReKi   ! line 561
```
Cable 使用相同模式——在 ExternInput 层存储 flat 1D 数组。

---

## 4. FAST_Subs.f90 — ExternInput → SrvD Input 数据传递

**文件**: `modules/openfast-library/src/FAST_Subs.f90`

### 4.1 SrvD_SetExternalInputs — 替换 placeholder (line 7982–7992)

```fortran
! 旧代码（placeholder 注释）:
!   ! StC controls
!   ! This is a placeholder for where StC controls would be passed ...

! 新代码:
if (ALLOCATED(u_SrvD%ExternalStCCmdStiff)) then
   do i=1,SIZE(u_SrvD%ExternalStCCmdStiff,2)
      u_SrvD%ExternalStCCmdStiff(1:3,i) = m_FAST%ExternInput%StCCmdStiff((i-1)*3+1:(i-1)*3+3)
      u_SrvD%ExternalStCCmdDamp( 1:3,i) = m_FAST%ExternInput%StCCmdDamp( (i-1)*3+1:(i-1)*3+3)
      u_SrvD%ExternalStCCmdBrake(1:3,i) = m_FAST%ExternInput%StCCmdBrake((i-1)*3+1:(i-1)*3+3)
      u_SrvD%ExternalStCCmdForce(1:3,i) = m_FAST%ExternInput%StCCmdForce((i-1)*3+1:(i-1)*3+3)
      u_SrvD%ExternalStCCmdMoment(1:3,i)= m_FAST%ExternInput%StCCmdMoment((i-1)*3+1:(i-1)*3+3)
   end do
end if
```

**作用**: 将 ExternInput 中的 flat 1D 数组（`StCCmdStiff(1:30)`）按通道展开为 ServoDyn 中的 2D 数组（`ExternalStCCmdStiff(1:3, i)`）。
- 通道 i 的 X 分量 = flat 数组位置 `(i-1)*3+1`
- 通道 i 的 Y 分量 = flat 数组位置 `(i-1)*3+2`
- 通道 i 的 Z 分量 = flat 数组位置 `(i-1)*3+3`

**参考**:
```fortran
! Cable controls — 1D→1D 直接复制
if (ALLOCATED(u_SrvD%ExternalCableDeltaL)) then
   do i=1,min(SIZE(u_SrvD%ExternalCableDeltaL),SIZE(m_FAST%ExternInput%CableDeltaL))
      u_SrvD%ExternalCableDeltaL(i) = m_FAST%ExternInput%CableDeltaL(i)  ! line 7972
   end do
end if
```
Cable 是 1D→1D，StC 是 1D→2D（增加了 reshape 逻辑）。外层 `if (ALLOCATED(...))` then 模式一致。

---

## 5. ServoDyn_Types.f90 — SrvD_InputType 及其配套操作

**文件**: `modules/servodyn/src/ServoDyn_Types.f90`

这是修改量最大的文件——StC 新增字段需要在整个类型生命周期中都有对应处理。

### 5.1 SrvD_InputType — 新增字段 (line 475–479)

```fortran
REAL(ReKi), DIMENSION(:,:), ALLOCATABLE :: ExternalStCCmdStiff
REAL(ReKi), DIMENSION(:,:), ALLOCATABLE :: ExternalStCCmdDamp
REAL(ReKi), DIMENSION(:,:), ALLOCATABLE :: ExternalStCCmdBrake
REAL(ReKi), DIMENSION(:,:), ALLOCATABLE :: ExternalStCCmdForce
REAL(ReKi), DIMENSION(:,:), ALLOCATABLE :: ExternalStCCmdMoment
```

**作用**: 在 SrvD 输入类型中存储从 Simulink 接收的 StC 控制指令。使用 `ALLOCATABLE` 而非固定大小——仅在用户启用 StC 外部控制时才分配，节省内存。

**参考**:
```fortran
REAL(ReKi), DIMENSION(:), ALLOCATABLE :: ExternalCableDeltaL     ! line 473
REAL(ReKi), DIMENSION(:), ALLOCATABLE :: ExternalCableDeltaLdot  ! line 474
```
StC 字段是 2D `(:,:)` 而 Cable 是 1D `(:)`，但 `ALLOCATABLE` 模式一致。

### 5.2 VarType 常量 (line 615–619)

```fortran
integer(IntKi), public, parameter :: SrvD_u_ExternalStCCmdStiff  = 55
integer(IntKi), public, parameter :: SrvD_u_ExternalStCCmdDamp   = 56
integer(IntKi), public, parameter :: SrvD_u_ExternalStCCmdBrake  = 57
integer(IntKi), public, parameter :: SrvD_u_ExternalStCCmdForce  = 58
integer(IntKi), public, parameter :: SrvD_u_ExternalStCCmdMoment = 59
```

**作用**: 为 StC 字段注册 VarType 标识符，供线性化、Pack/Unpack 等操作引用。

**为什么追加在末尾（55–59）而非插入中间？**
- 插入中间（如在 `SrvD_u_ExternalCableDeltaLdot=21` 之后插入为 22–26）会导致后续所有常量需要重新编号（`SrvD_u_TwrAccel` 从 22 变为 27，依此类推），风险极高。
- 追加在末尾（55–59）避免了重编号，且不影响功能——VarType 常量仅用于 select/case 分支，不要求数值连续。

**参考**: 已有 VarType 常量族 `SrvD_u_*` (line 563–619)，从 1 到 54。

### 5.3 SrvD_CopyInput — 复制逻辑 (line 4572–4639)

为每个 StC 字段新增复制块，每个块的形式为：
```fortran
if (allocated(SrcInputData%ExternalStCCmdStiff)) then
   LB(1:2) = lbound(SrcInputData%ExternalStCCmdStiff)
   UB(1:2) = ubound(SrcInputData%ExternalStCCmdStiff)
   if (.not. allocated(DstInputData%ExternalStCCmdStiff)) then
      allocate(DstInputData%ExternalStCCmdStiff(LB(1):UB(1), LB(2):UB(2)), stat=ErrStat2)
      if (ErrStat2 /= 0) then
         call SetErrStat(ErrID_Fatal, 'Error allocating DstInputData%ExternalStCCmdStiff.', ...)
         return
      end if
   end if
   DstInputData%ExternalStCCmdStiff = SrcInputData%ExternalStCCmdStiff
end if
```

**作用**: 在时间步之间复制 StC 输入数据。如果目标未分配则先分配（处理首次调用），然后整体赋值。

**与 Cable 的关键区别**: Cable 是 1D，使用 `LB(1:1)` / `UB(1:1)`。StC 是 2D，使用 `LB(1:2)` / `UB(1:2)`。

**参考**:
```fortran
! Cable — 1D 版本
if (allocated(SrcInputData%ExternalCableDeltaL)) then
   LB(1:1) = lbound(SrcInputData%ExternalCableDeltaL)
   UB(1:1) = ubound(SrcInputData%ExternalCableDeltaL)
   ...
end if
```

### 5.4 SrvD_DestroyInput — 销毁逻辑 (line 4743–4757)

```fortran
if (allocated(InputData%ExternalStCCmdStiff)) deallocate(InputData%ExternalStCCmdStiff)
! ... Damp, Brake, Force, Moment 同理
```

**作用**: 仿真结束时释放 StC 字段内存。

**参考**: Cable 的 `deallocate(InputData%ExternalCableDeltaL)` 和 `ExternalCableDeltaLdot`。

### 5.5 SrvD_PackInput / SrvD_UnpackInput — 打包/解包 (line 4821–4831, 5005–5015)

```fortran
! Pack
call RegPackAlloc(RF, InData%ExternalStCCmdStiff)
! ... Damp, Brake, Force, Moment

! Unpack
call RegUnpackAlloc(RF, OutData%ExternalStCCmdStiff); if (RegCheckErr(RF, RoutineName)) return
! ... Damp, Brake, Force, Moment
```

**作用**: 支持 checkpoint/restart 和线性化功能。`RegPackAlloc` 自动处理 allocatable 数组的打包（先记录是否已分配，再打包内容）。

**参考**: Cable 的 `call RegPackAlloc(RF, InData%ExternalCableDeltaL)` (line 4819–4820)。

### 5.6 SrvD_Input_ExtrapInterp1 / ExtrapInterp2 — 外推/内插 (line 6271–6302, 6426–6457)

```fortran
IF (ALLOCATED(u_out%ExternalStCCmdStiff) .AND. ALLOCATED(u1%ExternalStCCmdStiff)) THEN
   u_out%ExternalStCCmdStiff = a1*u1%ExternalStCCmdStiff + a2*u2%ExternalStCCmdStiff
END IF
```

**作用**: 在时间步之间进行外推/内插。`a1`, `a2`, `a3` 是时间权重系数。Fortran 数组运算 `a1*u1 + a2*u2` 直接作用于整个 2D 数组。

- `ExtrapInterp1`: 2 点线性外推/内插 (`a1*u1 + a2*u2`)
- `ExtrapInterp2`: 3 点二次外推/内插 (`a1*u1 + a2*u2 + a3*u3`)

**参考**: Cable 的对应代码块，格式完全一致。

### 5.7 VarType get / set / name — 线性化接口 (多处)

```fortran
! get — 从类型中读取值
case (SrvD_u_ExternalStCCmdStiff)
   if (allocated(u%ExternalStCCmdStiff)) &
      VarVals = reshape(u%ExternalStCCmdStiff, (/ size(u%ExternalStCCmdStiff) /))

! set — 向类型中写入值
case (SrvD_u_ExternalStCCmdStiff)
   if (allocated(u%ExternalStCCmdStiff)) &
      u%ExternalStCCmdStiff = reshape(VarVals, shape(u%ExternalStCCmdStiff))

! name — 返回字段名称
case (SrvD_u_ExternalStCCmdStiff)
    Name = "u%ExternalStCCmdStiff"
```

**作用**: 支持 OpenFAST 的线性化分析功能——允许外部工具通过 VarType 标识符读取/修改 StC 输入字段，用于计算状态空间矩阵。

**为什么使用 `reshape`？** VarVals 总是 1D 数组。对于 2D 字段，需要 `reshape` 进行维度转换：
- get: `reshape(2D_array, [total_size])` → 1D
- set: `reshape(1D_array, [dim1, dim2])` → 2D

**参考**: Cable 是 1D，不需要 reshape，直接 `VarVals = u%ExternalCableDeltaL(V%iLB:V%iUB)`。StC 因为 2D 额外需要 reshape。

---

## 6. ServoDyn.f90 — 核心逻辑

**文件**: `modules/servodyn/src/ServoDyn.f90`

### 6.1 SrvD_Init — 初始化分配 (line 465–480)

```fortran
if (p%NumStC_Control > 0 .and. p%StCCMode == ControlMode_EXTERN) then
   call AllocAry( u%ExternalStCCmdStiff,  3, p%NumStC_Control, ...)
   call AllocAry( u%ExternalStCCmdDamp,   3, p%NumStC_Control, ...)
   ! ... Brake, Force, Moment
   u%ExternalStCCmdStiff  = 0.0_ReKi
   ! ...
end if
```

**作用**: 仅在 StC 使用外部控制模式（`StCCMode == ControlMode_EXTERN`）时才分配 ExternalStC 数组。数组大小为 `(3, NumStC_Control)`，其中 `NumStC_Control` 由 `StC_CtrlChan_Setup` 在之前计算得出。

**为什么 DLL 模式不分配？**
- DLL 模式下 StC 指令来自 `m%dll_data%StCCmd*`，不需要 ExternalStC 数组
- 仅在 Simulink 外部控制模式下需要这些数组
- 节省内存

**参考**: Cable 的分配模式（line 447–462）：
```fortran
if (p%NumCableControl > 0) then
   call AllocAry( y%CableDeltaL, ... )
   call AllocAry( u%ExternalCableDeltaL, ... )
end if
```
Cable 没有模式判断（始终分配），因为 Cable 只有一种外部模式。StC 增加了 DLL vs EXTERN 的区分。

### 6.2 StC_CtrlChan_Setup — 控制模式检测 (line 1416–1431)

```fortran
! 旧代码:
if (p%BStC(i)%StC_CMode == ControlMode_DLL)  p%StCCMode = ControlMode_DLL

! 新代码:
if (p%BStC(i)%StC_CMode == ControlMode_DLL)     p%StCCMode = ControlMode_DLL
if (p%BStC(i)%StC_CMode == ControlMode_EXTERN)  p%StCCMode = ControlMode_EXTERN
```

**作用**: 扫描所有 StC 实例（Blade, Nacelle, Tower, Substructure），检测是否使用了外部控制模式。如果任何实例设置了 `StC_CMODE = 4`（EXTERN），则 `p%StCCMode` 被设为 `ControlMode_EXTERN`。

**模式优先级**: 按扫描顺序，后检测到的模式会覆盖之前的。当前逻辑下 DLL 和 EXTERN 都可以被检测到，但最后扫描到的会生效。未来如需混合模式（部分通道 DLL + 部分通道 EXTERN），需扩展为 per-channel 数组——这是代码中已有注释提到的已知限制（line 1404–1409）。

**参考**: 原有的 DLL 检测逻辑，增加了 EXTERN 并行检测。

### 6.3 StCControl_CalcOutput — 核心信号路由 (line 4606–4680)

这是最重要的修改——新增 EXTERN 分支。

#### 6.3.1 函数签名变更

```fortran
! 旧签名:
SUBROUTINE StCControl_CalcOutput(t, p, StC_CmdStiff, ..., m, ErrStat, ErrMsg)

! 新签名:
SUBROUTINE StCControl_CalcOutput(t, u, p, StC_CmdStiff, ..., m, ErrStat, ErrMsg)
```

**新增参数 `u`**: `TYPE(SrvD_InputType), INTENT(IN) :: u` — SrvD 输入，包含 `ExternalStCCmd*` 字段。

**参考**: `CableControl_CalcOutput` 不需要 u（Cable 数据直接从 y 输出），但 StC 需要 u 来读取 ExternalStC 字段。

#### 6.3.2 早期返回逻辑

```fortran
! 旧逻辑: DLL 必须配合扩展 avrSWAP 使用
if ((p%NumStC_Control <= 0) .or. (.not. p%EXavrSWAP)) return

! 新逻辑: EXTERN 不需要 EXavrSWAP
if ((p%NumStC_Control <= 0) .or. &
    (p%StCCMode /= ControlMode_DLL .and. p%StCCMode /= ControlMode_EXTERN)) return
if ((p%StCCMode == ControlMode_DLL) .and. (.not. p%EXavrSWAP)) return
```

**作用**: 
- 允许在 `StCCMode == ControlMode_EXTERN` 且 `EXavrSWAP == .false.` 时继续执行
- DLL 模式仍然需要 `EXavrSWAP == .true.`
- 如果 StCCMode 不是 DLL 也不是 EXTERN（如 NONE 或 Semi），则退出

#### 6.3.3 EXTERN 分支

```fortran
if (p%StCCMode == ControlMode_EXTERN) then
   if (allocated(StC_CmdStiff))  StC_CmdStiff(1:3,1:p%NumStC_Control)  = u%ExternalStCCmdStiff(1:3,1:p%NumStC_Control)
   if (allocated(StC_CmdDamp))   StC_CmdDamp( 1:3,1:p%NumStC_Control)  = u%ExternalStCCmdDamp( 1:3,1:p%NumStC_Control)
   if (allocated(StC_CmdBrake))  StC_CmdBrake(1:3,1:p%NumStC_Control)  = u%ExternalStCCmdBrake(1:3,1:p%NumStC_Control)
   if (allocated(StC_CmdForce))  StC_CmdForce(1:3,1:p%NumStC_Control)  = u%ExternalStCCmdForce(1:3,1:p%NumStC_Control)
   if (allocated(StC_CmdMoment)) StC_CmdMoment(1:3,1:p%NumStC_Control) = u%ExternalStCCmdMoment(1:3,1:p%NumStC_Control)
else
   ! DLL 分支（保持不变）
   if (p%DLL_Ramp) then ... else ... end if
end if
```

**作用**: 
- EXTERN 模式：直接从 `u%ExternalStCCmd*`（来自 Simulink）复制到局部 `StC_Cmd*` 数组。无 ramping（Simulink 自行管理平滑过渡）。
- DLL 模式：保持原有逻辑（从 `m%dll_data%StCCmd*` / `PrevStCCmd*` 读取，支持 ramping）。

**参考**: `CableControl_CalcOutput` 的 EXTERN 分支 (ServoDyn.f90:4545–4552)：
```fortran
CASE (ControlMode_EXTERN)
   if (allocated(u%ExternalCableDeltaL)) then
      CableDeltaL(1:p%NumCableControl) = u%ExternalCableDeltaL(1:p%NumCableControl)
   endif
```
完全相同的模式——EXTERN 路径直接复制，无额外处理。

#### 6.3.4 调用点更新

两处调用点都需要传入 `u` 参数：

```fortran
! SrvD_UpdateStates (line 1719):
call StCControl_CalcOutput(t_next, u_interp, p, StC_CmdStiff, ...)

! SrvD_CalcOutput (line 2011):
call StCControl_CalcOutput(t, u, p, StC_CmdStiff, ...)
```

**参考**: `CableControl_CalcOutput` 的调用方式，它不需要 u（Cable 从 m% 读取），因此 Cable 调用点不需要此改动。

---

## 7. StrucCtrl.f90 — 启用 CMODE_ActiveEXTERN

**文件**: `modules/servodyn/src/StrucCtrl.f90`

### 7.1 输入验证 (line 2295–2305)

```fortran
! 旧代码（CMODE_ActiveEXTERN 被注释掉）:
IF (InputFileData%StC_CMODE /= ControlMode_None .and. ... &
    InputFileData%StC_CMODE /= CMODE_ActiveDLL) &
    !InputFileData%StC_CMode /= CMODE_ActiveEXTERN .and. &  ! Not an option

! 新代码（启用 CMODE_ActiveEXTERN）:
IF (InputFileData%StC_CMODE /= ControlMode_None .and. ... &
    InputFileData%StC_CMODE /= CMODE_ActiveEXTERN .and. &
    InputFileData%StC_CMODE /= CMODE_ActiveDLL) &
```

**作用**: 允许用户在 StC 输入文件中设置 `StC_CMODE = 4`（Simulink 外部控制）。错误消息也相应更新，提示 4 是有效选项。

**注意**: `CMODE_ActiveEXTERN = 4 = ControlMode_EXTERN`（常量值相同，跨 StrucCtrl 和 ServoDyn 保持一致）。

### 7.2 控制通道范围检查 (line 2321)

```fortran
! 旧:
if (InputFileData%StC_CMode == CMODE_ActiveDLL) then

! 新:
if (InputFileData%StC_CMode == CMODE_ActiveDLL .or. &
    InputFileData%StC_CMode == CMODE_ActiveEXTERN) then
```

**作用**: EXTERN 模式下同样需要检查 `StC_CChan` 是否在有效范围（0–10）。

### 7.3 状态导数计算 (line 1228)

```fortran
! 旧:
ELSE IF (p%StC_CMODE == CMODE_ActiveDLL) THEN

! 新:
ELSE IF (p%StC_CMODE == CMODE_ActiveDLL .or. &
         p%StC_CMODE == CMODE_ActiveEXTERN) THEN
```

**作用**: EXTERN 和 DLL 使用相同的 `StC_ActiveCtrl_StiffDamp` 函数。这是本次设计的关键——**EXTERN 和 DLL 在 StC 模块内部走相同路径**，区别仅在于上层 ServoDyn 如何填充 `u%Cmd*` 数组（DLL → 从 Bladed DLL 获取，EXTERN → 从 Simulink InputAry 获取）。

**参考**: `StC_ActiveCtrl_StiffDamp` (StrucCtrl.f90:1730–1755) 从 `u%CmdStiff/CmdDamp/CmdBrake/CmdForce/CmdMoment` 读取控制值——无论这些值来源如何，函数行为一致。

### 7.4 DOF/Mesh 模式检查 (line 2308)

```fortran
if (InputFileData%StC_CMode == CMODE_ActiveUsrSub .or. &
    InputFileData%StC_CMode == CMODE_ActiveDLL    .or. &
    InputFileData%StC_CMode == CMODE_ActiveEXTERN) then
```

**作用**: EXTERN 模式与 DLL/UserSub 模式一样，仅支持 independent、omni-directional DOF 或 ForceDLL 模式。

### 7.5 StC_SetParameters — 通道赋值 (line 2585)

```fortran
! 旧:
if (p%StC_CMODE == CMODE_ActiveDLL) then

! 新:
if (p%StC_CMODE == CMODE_ActiveDLL .or. p%StC_CMODE == CMODE_ActiveEXTERN) then
```

**作用**: EXTERN 模式下保留 StC_CChan 配置（不从输入文件清零），因为 EXTERN 仍然需要通道号来对应 InputAry 中的位置。

---

## 8. ServoDyn_IO.f90 — 摘要输出

**文件**: `modules/servodyn/src/ServoDyn_IO.f90`

### 8.1 WrSumInfo4Simulink (line 2484–2529)

```fortran
! 新增 StC 状态行:
call WrCtrlInfo(p%StCCMode == SimulinkCtrlMode, 'StC active control   ',
                '(StCCmdStiff,Damp,Brake,Force,Moment)')

! 更新 InputAry 文档表格 (新增 52–201):
write(UnSum, '(8x,A5,4x,A3,3x,A)')  '52:81 ','<--','StCCmdStiff            '
write(UnSum, '(8x,A5,4x,A3,3x,A)')  '82:111','<--','StCCmdDamp             '
write(UnSum, '(8x,A5,4x,A3,3x,A)')  '112:141','<--','StCCmdBrake            '
write(UnSum, '(8x,A5,4x,A3,3x,A)')  '142:171','<--','StCCmdForce            '
write(UnSum, '(8x,A5,4x,A3,3x,A)')  '172:201','<--','StCCmdMoment           '
```

**作用**: 在 ServoDyn 摘要文件（`*.sum`）中打印 Simulink InputAry 布局，帮助用户调试。如果 StC 使用 Simulink 控制，则显示 "StC active control" 行。

**参考**: Cable 的摘要行和已有的 InputAry 表格格式。

---

## 信号流总结

```
┌─────────────────────────────────────────────────┐
│ Simulink InputAry                                │
│ pos 52-81:  StCCmdStiff(30)                      │
│ pos 82-111: StCCmdDamp(30)                       │
│ pos 112-141: StCCmdBrake(30)                     │
│ pos 142-171: StCCmdForce(30)                     │
│ pos 172-201: StCCmdMoment(30)                    │
└────────────────┬────────────────────────────────┘
                 │ FAST_SetExternalInputs (FAST_Library.f90)
                 │ flat 1D → flat 1D (直接切片)
                 ▼
┌─────────────────────────────────────────────────┐
│ FAST_ExternInputType (FAST_Types.f90)            │
│ StCCmdStiff(1:30), StCCmdDamp(1:30), ...         │
└────────────────┬────────────────────────────────┘
                 │ SrvD_SetExternalInputs (FAST_Subs.f90)
                 │ flat 1D → 2D reshape
                 ▼
┌─────────────────────────────────────────────────┐
│ SrvD_InputType (ServoDyn_Types.f90)              │
│ ExternalStCCmdStiff(1:3, 1:NumStC_Control)       │
│ ExternalStCCmdDamp(1:3, 1:NumStC_Control)        │
│ ...                                              │
└────────────────┬────────────────────────────────┘
                 │ StCControl_CalcOutput (ServoDyn.f90)
                 │ ControlMode_EXTERN 分支
                 ▼
┌─────────────────────────────────────────────────┐
│ 局部变量 StC_CmdStiff/Damp/Brake/Force/Moment    │
└────────────────┬────────────────────────────────┘
                 │ SetStCInput_CtrlChans (ServoDyn.f90)
                 ▼
┌─────────────────────────────────────────────────┐
│ StC_InputType%CmdStiff/Damp/Brake/Force/Moment   │
└────────────────┬────────────────────────────────┘
                 │ StC_ActiveCtrl_StiffDamp (StrucCtrl.f90)
                 │ (DLL 和 EXTERN 共用此路径)
                 ▼
┌─────────────────────────────────────────────────┐
│ K_ctrl, C_ctrl, C_Brake, F_ctrl, M_ctrl          │
│ → StC 设备物理方程计算                            │
└─────────────────────────────────────────────────┘
```