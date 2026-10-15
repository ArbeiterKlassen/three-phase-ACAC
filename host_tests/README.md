# 宿主机测试与仿真

这里的代码只编译 `App/Inc` 与 `App/Src`，不依赖 HAL、CMSIS 或 STM32 寄存器，
所以能在 PC 上直接跑。构建产物写入系统临时目录，不会污染 CubeIDE 工程。

## 回归测试（18 项）

```powershell
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1
```

脚本优先使用 PATH 中的 GCC；没有 GCC 时回退到已安装的 Visual Studio C/C++
工具集。覆盖范围：

- ADC 解包（rank 映射）与四路标定、`Ic/Vca/三相相电压` 重构；
- 无查表相位递推的周期与幅值；
- Tustin QPR 的谐振增益与限幅抗饱和；
- SVPWM：三相占空比关系、过调制等比例缩放；
- 逆变电压外环/电流内环级联、开环诊断通道；
- 保护：测量无效确认窗口、相过流、输出过压、采样停滞、锁存与清除；
- 状态机全流程与 `pwm_allowed` 门控；
- 板级配置校验（母线/输出参考窗口、QPR 增益、频率白名单、各确认标志）；
- 单例 API：Vdc 设定、输出微调、频率切换、故障注入与复位；
- 输出过压故障快照（含"停车后不携带陈旧历史"的守卫）。

## 闭环仿真

```powershell
gcc -std=c11 -I App\Inc host_tests\sim_inverter.c App\Src\*.c -lm -o sim_inv.exe
.\sim_inv.exe 1        # 1 = PC4 接通，0 = PC4 悬空（本工程不采母线，两者等价）
```

被控对象模型：三相全桥（用占空比直接产生桥臂电压）+ LC 滤波器 + 星形阻性负载。
控制器用的是真实源码，只有对象是简化模型。

基线数字（改控制参数后必须对得上）：

```
稳态 |vab| 峰值 45.20 V   参考 32.00 V(rms 线)   state=4(RUN)   faults=0
```

## 备注

- 采样帧是四个 32 位 CDR 字：每字低 16 位 ADC1、高 16 位 ADC2；
  rank 依次为 `Iin/Vin`、`Ib/Ia`、`Vab/Vbc`、`Vdc/Vin诊断`。
  **本工程只解包 rank2 与 rank3**，rank1/rank4 保留在 DMA 序列里维持时序。
- 注入故障快照时用的原始码是按标定反算出来的，改标定要同步改这些期望值。
