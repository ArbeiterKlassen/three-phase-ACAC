#pragma once

/*
 * 工程常用常数（唯一真源）。
 *
 * 规则：算法里要用 π、√2、√3、1/√3 这些常数，**直接用这里的宏**，
 * 不要在源文件里再抄一遍字面量，也不要各模块自己 define 一份。
 *
 * 命名：APP_ 前缀 + 数学写法。1DIV = 倒数，SQRT = 平方根，DIV = 除。
 *   例：APP_1DIVSQRT3_F = 1/√3，APP_SQRT2DIV2_F = √2/2。
 * 后缀 _F 表示 float 常量（f 后缀的浮点字面量），可以直接参与 float 运算。
 */

#define APP_PI_F (3.14159265358979323846f)            /* 3.14159... */
#define APP_2PI_F (6.28318530717958647692f)           /* 2π */
#define APP_1DIV2PI_F (0.15915494309189533577f)       /* 1/(2π) */
#define APP_1DIVSQRT3_F (0.57735026918962576451f)     /* 1/√3，线电压↔相电压换算用 */
#define APP_SQRT3_F (1.73205080756887729353f)         /* √3 */
#define APP_SQRT3DIV2_F (0.86602540378443864676f)     /* √3/2 */
#define APP_SQRT2DIV2_F (0.70710678118654752440f)     /* √2/2 */
#define APP_SQRT2DIV3_F (0.47140452079103168293f)     /* √2/3 */
#define APP_SQRT2DIV6_F (0.23570226039551584146f)     /* √2/6 */
#define APP_1DIVSQRT2_F (0.70710678118654752440f)     /* 1/√2 */
#define APP_1DIVSQRT2DIV3_F (0.23570226039551584146f) /* 1/(√2·3) */
#define APP_1DIVSQRT2DIV6_F (0.11785113019775792073f) /* 1/(√2·6) */
#define APP_SQRT2DIV3DIV2_F (0.23570226039551584146f) /* √2/(3·2) */
#define APP_SQRT2DIV3DIV4_F (0.11785113019775792073f) /* √2/(3·4) */
#define APP_SQRT2_F (1.41421356237309504880f)         /* √2，相电压 RMS→峰值 */
