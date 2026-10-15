# 代码规范

这份规范从 `CompVision-Assignment`（C++/MFC）的写法平移而来。原工程是 C++，
这里因地制宜换成 C 的等价写法，**风格意图保持一致**。

---

## 1. 模块划分

一个模块 = 一对 `.h/.c`，职责单一，名字直接说明它管什么。

```
measurement.c/h       ADC 解包与标定
protection.c/h        故障判断与锁存
svpwm.c/h             空间矢量调制
```

不使用 `utils`、`common`、`misc` 这类含糊名字——那是没想清楚职责的信号。

---

## 2. 命名

| 对象 | 规则 | 例子 |
|---|---|---|
| 模块对外函数 | `模块_动词`（C 里对应 C++ 的 namespace） | `Measurement_Convert()` |
| 模块内部函数 | `static` + 同样规则 | `static float Clamp()` |
| 类型 | 大驼峰，名词短语 | `Measurements`、`ProtectionConfig` |
| 结构体字段 | 小写下划线，带单位后缀 | `dc_voltage_v`、`sample_time_s` |
| 宏常量 | 全大写，带模块前缀 | `PROTECTION_FAULT_MEASUREMENT` |
| 布尔量 | `is_` / `has_` / 过去分词 | `is_ready`、`valid`、`enabled` |

**单位写进名字**：`_v` 伏、`_a` 安、`_s` 秒、`_hz` 赫兹、`_v_per_s` 斜率。
凡是数值，名字里必须能看出单位。这条比什么都重要——省赛现场的量纲错误
几乎都出在没有单位后缀上。

---

## 3. 注释

### 3.1 头文件：每个声明一行中文

```c
// 把原始 ADC 帧换算为物理量，并重构第三相
// raw 必须来自本帧解包；返回 false 时 out 被清零
bool Measurement_Convert(const AdcRawFrame *raw, Measurements *out);
```

**说清三件事**：做什么、参数有什么前提、返回值什么含义。
不写"函数名顾名思义"这类废话。

### 3.2 结构体：每个字段行尾注释

```c
/* 单通道标定。物理量 = (原始码 − offset_count) × gain。 */
typedef struct {
    float offset_count; // 零输入对应的 ADC 原始码
    float gain;         // 每码对应的物理量（含传感器极性、变比与分压比）
    uint16_t raw_min;   // 合法原始码下限；超出判"测量无效"
    uint16_t raw_max;   // 合法原始码上限
} ChannelCalibration;
```

行尾注释的对齐由工具负责（见第 7 节），不要手工敲空格去对齐。

### 3.3 源文件：解释"为什么"，不复述"做什么"

```c
/* 上电时母线电容的充电浪涌会让电流通道短暂越限。窗口放宽到 100 ms：
   浪涌持续不到 1 ms，过后计数器清零；传感器真断线会一直无效，照常报故障。 */
```

代码已经说清了"做什么"，注释只补**代码里看不出来的东西**：
为什么是这个数、为什么不能那样写、物理上发生了什么。

### 3.4 文件头

只写一段简短中文说明职责，**不写版权套话、不写修改历史**：

```c
/*
 * 故障判断、确认窗口与锁存。
 * 硬件 Break 由定时器直接清 MOE，这里处理软件判据与状态恢复。
 */
```

---

## 4. 头文件保护

用 `#pragma once`（与参考工程一致），不再用 `#ifndef XXX_H` 三段式。

---

## 5. 函数

- 单一职责。**超过 100 行先考虑拆分**。
- 参数不超过 5 个；超了就打包成结构体。
- 返回 `bool` 表示"是否有效/是否成功"，错误原因通过输出参数带回。
- 不做隐式单位换算；换算写在名字里（`Convert`、`ToCompare`）。

---

## 6. 分层

```
Core/       板级：寄存器、外设、DMA、中断。不写控制算法。
App/        算法：纯计算，不碰寄存器，可在 PC 上编译运行。
host_tests/ 验证：只依赖 App/，不依赖 HAL。
```

**这条分层是硬约束**，也是 `host_tests` 能跑起来的原因。
往 `App/` 里加 HAL 调用会立刻破坏离线测试能力。

---

## 7. 版式

版式**交给工具**，不手工对齐：仓库根目录的 `.clang-format` 是唯一真源，
改它等于改这一节。

```powershell
# 系统里没有就装一个（官方发布的二进制包）
python -m pip install clang-format

# 格式化改过的文件，或整个仓库
clang-format -i App\Src\rectifier_control.c
clang-format -i App\Inc\*.h App\Src\*.c Core\Src\main.c Core\Src\board_runtime.c
```

| 项 | 规定 |
|---|---|
| 缩进 | 4 空格，不用 Tab |
| 大括号 | 跟随语句：`if (x) {`、`void f() {`、`typedef struct {` |
| 单语句分支 | 省略大括号，可压成一行：`if (x) a; else b;` |
| 简单限幅 | 允许三目：`return v < lo ? lo : (v > hi ? hi : v);` |
| 行宽 | 120 列；超了自动折行（不手工折行、也不手工拼长行） |
| 注释 | 原样保留不重排（`ReflowComments: false`），行尾注释自动对齐 |
| include | 不排序（`SortIncludes: Never`），保留人为顺序 |
| 函数体 | 单条语句的小函数允许写成一行（`AllowShortFunctionsOnASingleLine`） |

两条配套纪律：

1. **单语句省大括号是允许的，但加第二句时必须自己补大括号。**
   `.clang-format` 里开了 `InsertBraces`，它只在"不加就会改变语义"时兜底，
   救不了写错的缩进意图——加语句时顺手把大括号写上。
2. **不为对齐手工敲空格**。对齐与折行都由工具决定，手敲的下次格式化就没，
   还会在 diff 里制造噪声。

> 这套版式与 `CompVision-Assignment` 的 C++ 风格一致：大括号跟随语句、
> 单语句分支不加括号、工具负责对齐。

---

## 8. 交给下一届的三条

1. **改控制参数前先读 `app_config.c` 顶部的注释**，那里写了每个数的来历。
2. **改代码后先跑 `host_tests`**，30 项挂了任何一项都别烧板子。
3. **改完代码跑一次 `clang-format -i`**（或让 IDE 保存时自动格式化），
   别让版式散开。
4. **不确定就写下来**。注释里留一句"这里假设 XXX"，比留一个自以为是的实现有用得多。
