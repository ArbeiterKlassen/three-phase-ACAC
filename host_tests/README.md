# 应用层宿主机测试

这些测试只编译 `App/Inc` 与 `App/Src`，不依赖 HAL、CMSIS 或 STM32
寄存器。构建产物写入系统临时目录，不会修改 CubeMX 工程文件。

在工程根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1
```

脚本优先使用 PATH 中的 GCC；未找到 GCC 时，会使用已安装的 Visual
Studio C/C++ 工具集。测试覆盖：

- 七路 ADC 标定与 `Ic/Vca/三相相电压` 重构；
- 无查表相位递推的周期和幅值；
- PI 抗饱和、Tustin QPR 谐振增益、100 Hz 陷波；
- SVPWM 线电压关系、过调制缩放、整流全桥单极性 SPWM；
- 逆变电压外环/电流内环级联；
- 整流 400 点真 RMS、`Vin/Vrms` 同相电流参考、1 kHz 慢环和 Vdc 软启动；
- MODE2 单级、MODE0 连续 1000 拍母线确认及确认中断后重新计数；
- 保护锁存和完整状态机；
- 按模式配置、采样量程无效时 RUN/PWM 的硬锁止。

## 接入前必须完成

当前配置使用已确认的同型采样模块标定，并由
`BoardPower_ValidateModeConfig()` 同时检查偏置、增益、极性、控制限幅、
软件保护阈值和可测量程。板级适配可覆盖弱符号
`BoardPower_GetConfig()`，但不能绕过集中安全校验。

双 ADC `CDR` 帧固定为四个 32 位字：每字低 16 位是 ADC1，高 16 位是
ADC2；rank 依次为 `Iin/Vin`、`Ib/Ia`、`Vab/Vbc`、`Vdc/Vin诊断`。
