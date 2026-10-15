# 错误记录与分析（交组员）

本文记录三相逆变（DCAC）工程里**已定位并修复的问题**，以及**尚未复现、
需要上板确认**的疑点。所有结论都附可复现的证据（命令或仿真输出）。

---

## 一、已确证并修复

### E1 · 主机测试在 Windows 上无法链接（阻塞全部离线验证）

**症状**

```
undefined reference to `BoardPower_GetConfig'
```

**影响**：`host_tests/` 是这套代码唯一的离线验证手段。它跑不起来，意味着
之后的任何改动都没有经过测试。

**根因**：`app_config.c` 用弱符号提供板级配置的默认实现：

```c
#if defined(__GNUC__) && !defined(_WIN32)
#define APP_WEAK __attribute__((weak))
```

弱符号在 ELF（`arm-none-eabi`，即单片机目标）上是一等公民，链接器直接支持；
但 PE/COFF 没有弱绑定，GCC 只能用 `.weak.<符号>.<目标>` 别名段模拟，而 MinGW
的 binutils 不解析该段。`nm` 证据：

```
w BoardPower_GetConfig                              ← 弱引用，未定义
T .weak.BoardPower_GetConfig.g_board_power_config   ← 别名段
```

**修复**：`_WIN32` 下退化为普通强定义（ELF 目标仍保留弱符号覆盖能力）。

**验证**：`powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1`
→ 18 项全部通过。

---

## 二、尚未复现，需要上板确认

### Q1 · 输出只有 3 V 多，且每隔几秒出现一次频率震荡

**现场描述**：母线 >60 V 直入（整流级完全断开），输出频率正确，但峰值只有
3 点几 V（目标 45.3 V 峰值），并每隔几秒出现一次频率震荡。

**仿真结论：控制器本身是对的。**

用 `host_tests/sim_inverter.c`（三相全桥 + LC + 星形负载闭环，母线 61 V 直入）：

```
  0.20  RUN  |vab|峰值  8.96 V   ← 软启动爬升（约 1 s）
  1.00  RUN  |vab|峰值 45.47 V
  1.20~3.00     45.20 V          ← 稳态 = 32 Vrms × √2 = 45.25 V ✓
结束: state=RUN  faults=0x00000000
```

PC4 接通与悬空结果完全一致——因为本工程的调制分母取用户 SET 值
（见 README 第 6 节），与是否接母线采样无关。

**所以 3 V 不是控制算法算出来的，更像“桥臂根本没在开关”。** 按可疑度排序：

| # | 假设 | 判别方法 |
|---|---|---|
| 1 | **栅极驱动使能（EN/SD）没被拉高**。原理图里把 `EN_INV` 列为“尚未分配”，若驱动板需要使能信号而固件没给，MOSFET 永不导通 | 上电后直接量驱动板 EN/SD 引脚电平；示波器看**栅极**（不是 MCU 引脚）有无 20 kHz 波形 |
| 2 | 母线实际电压与 SET 严重不符。调制分母用的是 SET 值，两者差得远会让占空比整体错位 | 记录 OLED 上的 SET 与实际母线电压；用 SET=实测值重启对比 |
| 3 | 输出侧接的不是预期的滤波/负载，或电压采样在滤波器前/后不一致 | 示波器直接看桥臂中点、再看滤波后，逐级定位衰减发生在哪一级 |
| 4 | 使能周期性丢失导致软启动反复重启（周期约 1 s，接近“每隔几秒”） | OLED 盯 `state`；或把 `reference_slew_v_per_s` 临时调大，看输出是否跟着变大 |

**已排除**：控制器增益不足（空载衰减说明不是增益问题）、参考被
`LimitPhaseReference` 削顶（61 V 母线时上限 23.1 V，参考只有 18.5 V）、
SVPWM 过调制缩放（占空比 0.13~0.87，在 [0.02, 0.98] 内）。

---

## 三、仿真怎么跑

```powershell
# 回归测试（18 项）
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1

# 闭环仿真（三相桥 + LC + 星形负载，母线 61 V 直入）
gcc -std=c11 -I App\Inc host_tests\sim_inverter.c App\Src\*.c -lm -o sim_inv.exe
.\sim_inv.exe 1      # 1 = PC4 接通，0 = PC4 悬空
```

仿真接**真实的控制器源码**，只有被控对象是简化模型（三相全桥 + LC 滤波器 +
星形阻性负载）。被控对象参数（电感、电容、负载）是**假设值**——原理图上的
实际值应替换后重跑，结论会更准。

---

## 四、变更记录

| 提交 | 内容 |
|---|---|
| `baseline` | 收录 8/1 整机版本，建立 git 基线 |
| `fix: 上电浪涌导致测量故障误锁存` | 整机工程时期修复（含 E1 的链接修复） |
| `refactor: 全仓库统一版式` | 引入 `.clang-format`，统一代码风格 |
| `refactor: 常用常数集中到 constants.h` | 数学常数唯一真源 |
| `refactor: DCAC 独立工程` | 拆出本分支：移除 ACDC 模块与多模式逻辑，行为与整机 MODE1 逐位一致 |
