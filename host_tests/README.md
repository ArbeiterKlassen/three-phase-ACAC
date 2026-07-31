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
- 逆变五通道（含实测 Vdc）与整流三通道（三相置 0）两种换算路径；
- 无查表相位递推的周期和幅值；
- PI 抗饱和、Tustin QPR 谐振增益、100 Hz 陷波；
- SVPWM 线电压关系、过调制缩放、整流全桥单极性 SPWM；
- SVPWM 对固定及 100 Hz 纹波 Vdc 的 `1/vdc` 归一化，验证无长周期
  幅值包络；
- MODE2 固定 Vdc 闭环（RL 负载模型、延迟一周期装载）多周期包络回归；
- 逆变电压外环/电流内环级联；
- 整流实测 `Vin/Vrms` 同相电流参考、1 kHz 慢环和 Vdc 软启动；
- 保护锁存和完整状态机；
- 三模式配置校验（`BoardPower_ValidateModeConfig`）与
  `BoardPower_ConfigValid()==false` 时 RUN/PWM 的硬锁止；
- MODE1 整流单级与 MODE0 整流→逆变顺序开放的完整应用流程。

## 接入前必须完成

`App/Src/app_config.c` 中的 Ia/Ib/Vab/Vbc/Vdc 五路已写入实测标定；
Iin/Vin 临时复用同型 TA/TV 通道的标定参数（**更换前端必须重新标定**），
整流 PI/QPR 为保守可运行值。实机接入时仍必须确认采样增益、极性、
保护阈值、桥臂极性和死区。板级适配可覆盖弱符号
`BoardPower_GetConfig()`，但不能绕过 `BoardPower_ConfigValid()` 的
集中安全校验；只改 `config_valid` 仍会因零增益/非法参数校验而保持锁止。

双 ADC `CDR` 帧固定为四个 32 位字：每字低 16 位是 ADC1，高 16 位是
ADC2；rank 依次为 `Iin/Vin`、`Ia/Ib`、`Vab/Vbc`、`Vdc/Vin诊断`。
