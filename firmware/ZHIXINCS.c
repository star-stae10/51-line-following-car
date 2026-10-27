/*
 * ZHIXINCS.C - 循迹 + 超声波避障（低速 PWM 版）· 注释版
 * 主控：STC89C52RC    晶振：11.0592MHz    12T
 *
 * 【本文件说明】由 ZHIXINCS程序说明.md 的代码块导出，UTF-8 编码，
 *   供 VSCode 阅读和对照。用于 Keil 编译的原 ZHIXINCS.C 是 GBK 编码，
 *   注释内容以本版为准（本版注释更全）。直接把本文件放进 Keil 编译
 *   中文注释会乱码：要么把文件另存为 GBK/ANSI，要么在
 *   Edit -> Configuration -> Editor -> Encoding 里选 UTF-8。
 *
 * ===== 接线 =====
 *   左前 P2.3/P2.2   左后 P2.1/P2.0   (L298N①)
 *   右前 P3.7/P3.6   右后 P3.5/P3.4   (L298N②)
 *   左循迹 P2.4      右循迹 P2.5
 *   Trig P3.3        Echo P3.2
 *   PWM: P1.6 = 模块① ENA（左前）
 *        P1.7 = 模块① ENB（左后）
 *        P1.5 = 模块② ENA（右前）
 *        P1.4 = 模块② ENB（右后）
 *   ENA/ENB 跳线帽必须拔掉，否则 P1 与 5V 对顶。
 *
 * ===== 速度设定（占空比，不是转速百分比）=====
 *   CRUISE 35%   正常循迹
 *   OUTER  40%   转向时外侧轮
 *   INNER  25%   转向时内侧轮
 *   PIVOT  35%   原地转向 / 找线
 *
 *   占空比不等于转速：TT 马达有启动死区，20% 以下常常转不动，
 *   25% 是能稳定转动的最低档附近。若内侧轮 25% 不转，把
 *   INNER_DUTY 提到 30%；若整车起步困难，把 CRUISE_DUTY 提到 40%。
 *
 * ===== 11.0592MHz 计算依据 =====
 *   机器周期 = 12 / 11059200 = 1.08507 us
 *   1 ms     = 921.6 个机器周期（取 922）
 *   50 ms    = 46080 个机器周期
 *
 *   Timer1 方式2：TH1 = 72，每 184 个机器周期溢出一次
 *                 184 x 1.08507 = 199.65 us -> 5008.8 Hz
 *   PWM 20 级    -> 5008.8 / 20 = 250.4 Hz
 *   占空比分辨率 1/20 = 5%
 *
 *   超声波：Echo 高电平持续 t 个机器周期
 *           距离 = t x (12 / 11059200) x 34000 / 2   (cm)
 *                = t x 204000 / 11059200
 *                = t x 204 / 11059                   (cm)
 *           20cm 对应 t = 1085；5ms 对应 4608 周期，约 85cm
 *
 * ===== 时钟模式（烧录时必须选 12T！）=====
 *   下面所有数字只对 12T 成立。STC89C52RC 支持 6T/12T 切换，
 *   在 STC-ISP 下载软件里选，代码和 Keil 工程都管不了它。
 *   误选 6T 的三种后果，方向各不相同：
 *     1) delay_ms 实际延时减半 -> 主循环快一倍，车约快一倍；
 *     2) PWM 中断约 10kHz，输出约 500.9Hz（占空比数值不变）；
 *     3) 测距计数 t 翻倍 -> 距离读数翻倍，15cm 阈值实际 7.5cm
 *        才触发，看着正常直到撞上。换芯片/换电脑后先核对此项。
 *
 * ===== 停车距离 =====
 *   低速行驶 + 短接制动，从判定到停住只多走 1~2cm，属于可接受
 *   范围，因此不再做刹车距离补偿：判定阈值就是 15cm。
 *   若实测偏差超过 3cm，把 BRAKE_TRAVEL_CM 改成实测值。
 */

#include <reg52.h>
#include <intrins.h>

/* ---------- 晶振与时间基准 ---------- */
#define FOSC_HZ             11059200UL

#define TICKS_PER_MS        ((FOSC_HZ + 6000UL) / 12000UL)         /* 922 */
#define TICKS_50MS          ((FOSC_HZ * 50UL + 6000UL) / 12000UL)  /* 46080 */
#define RELOAD_1MS          (65536UL - TICKS_PER_MS)
#define RELOAD_50MS         (65536UL - TICKS_50MS)

/* ---------- PWM ---------- */
#define PWM_LEVELS          20
#define PWM_MAX             PWM_LEVELS
#define DUTY_PERCENT(p)     ((unsigned char)(((unsigned int)(p) * PWM_LEVELS + 50U) / 100U))

#define CRUISE_DUTY         DUTY_PERCENT(35)    /* 7/20 */
#define OUTER_DUTY          DUTY_PERCENT(40)    /* 8/20 */
#define INNER_DUTY          DUTY_PERCENT(25)    /* 5/20 */
#define PIVOT_DUTY          DUTY_PERCENT(35)    /* 7/20 */

/* ---------- 避障距离 ---------- */
#define TARGET_STOP_CM      15
#define BRAKE_TRAVEL_CM     0
#define PROBE_OFFSET_CM     0
#define STOP_DISTANCE_CM    (TARGET_STOP_CM + PROBE_OFFSET_CM + BRAKE_TRAVEL_CM)
#define RESUME_DISTANCE_CM  (STOP_DISTANCE_CM + 5)
#define RESUME_CONFIRM      3
#define FAULT_CONFIRM       3
#define FAULT_RECOVER       3

/* ---------- 循迹与找线 ---------- */
#define LINE_BLACK_LEVEL    1       /* 黑线输出高；批次相反时改 0 */
#define LOOP_MS             5
#define MEASURE_LOOPS       20      /* 100ms 测距周期，满足 >=60ms */

/*
 * 丢线找回：
 *   1. 先保持方向直行 SEARCH_WALK_MS，救回黑线断口 / 十字路口；
 *   2. 再向最后看到线的方向连续原地转，最多一圈加 10% 余量。
 *      PIVOT_90_MS 是 35% 占空比下原地转 90 度所需时间，需实测标定。
 */
#define SEARCH_WALK_MS      150
#define PIVOT_90_MS         1100
#define SEARCH_TOTAL_MS     (PIVOT_90_MS * 4 + PIVOT_90_MS * 4 / 10)

/* ---------- 超声波 ---------- */
#define ECHO_HIGH_LIMIT     3
#define ECHO_RISE_TIMEOUT   1000
#define ECHO_MAX_TICKS      ((FOSC_HZ * 5UL + 6000UL) / 12000UL)    /* 5ms 约 85cm */

/* ---------- 测距返回值 ---------- */
#define DIST_INVALID        0xFFFF  /* 模块无响应 / 断线 */
#define DIST_FAR            0xFFFE  /* 无障碍 / 超量程 */
#define DIST_BLIND          0xFFFD  /* 盲区（<2cm），按极近障碍处理 */

/* ---------- 状态机 ---------- */
#define STATE_FOLLOW        0
#define STATE_SEARCH        1
#define STATE_OBSTACLE      2
#define STATE_LOST          3
#define STATE_FAULT         4

/* ---------- 引脚 ---------- */
sbit TRIG = P3^3;
sbit ECHO = P3^2;
sbit LEFT_SENSOR  = P2^4;
sbit RIGHT_SENSOR = P2^5;

sbit LF_IN1 = P2^3;
sbit LF_IN2 = P2^2;
sbit LR_IN1 = P2^1;
sbit LR_IN2 = P2^0;

sbit RF_IN1 = P3^7;
sbit RF_IN2 = P3^6;
sbit RR_IN1 = P3^5;
sbit RR_IN2 = P3^4;

sbit PWM_LF = P1^6;    /* 模块① ENA，左前 */
sbit PWM_LR = P1^7;    /* 模块① ENB，左后 */
sbit PWM_RF = P1^5;    /* 模块② ENA，右前 */
sbit PWM_RR = P1^4;    /* 模块② ENB，右后 */

/* L298N：IN1/IN2 = 1/0 正转，0/1 反转，0/0 或 1/1 停 */
#define LEFT_FWD()   do { LF_IN1 = 1; LF_IN2 = 0; LR_IN1 = 1; LR_IN2 = 0; } while (0)
#define LEFT_REV()   do { LF_IN1 = 0; LF_IN2 = 1; LR_IN1 = 0; LR_IN2 = 1; } while (0)
#define RIGHT_FWD()  do { RF_IN1 = 1; RF_IN2 = 0; RR_IN1 = 1; RR_IN2 = 0; } while (0)
#define RIGHT_REV()  do { RF_IN1 = 0; RF_IN2 = 1; RR_IN1 = 0; RR_IN2 = 1; } while (0)
#define LEFT_STOP()  do { LF_IN1 = 0; LF_IN2 = 0; LR_IN1 = 0; LR_IN2 = 0; } while (0)
#define RIGHT_STOP() do { RF_IN1 = 0; RF_IN2 = 0; RR_IN1 = 0; RR_IN2 = 0; } while (0)

/* ---------- 全局变量 ---------- */
static volatile unsigned char pwm_tick = 0;
static volatile unsigned char duty_lf = 0;
static volatile unsigned char duty_lr = 0;
static volatile unsigned char duty_rf = 0;
static volatile unsigned char duty_rr = 0;
static unsigned char echo_high_count = 0;

/* ==================================================================
 * 初始化
 * ================================================================== */

/* Timer0 方式1：16 位定时，用于延时和超声波计时 */
void timer0_init(void)
{
    TMOD &= 0xF0;
    TMOD |= 0x01;
    TH0 = 0;
    TL0 = 0;
    TF0 = 0;
    TR0 = 0;
}

/*
 * Timer1 方式2：8 位自动重装，TH1 = 72
 * 11.0592MHz/12T：184 个机器周期 = 199.65us -> 约 5kHz
 * PWM 20 级 -> 250.4Hz
 */
void pwm_init(void)
{
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1 = 72;
    TL1 = 72;
    ET1 = 1;
    EA = 1;
    TR1 = 1;
}

/* ==================================================================
 * 定时器 1 中断：4 路软件 PWM
 *   P1.6 左前、P1.7 左后、P1.5 右前、P1.4 右后
 *   P1.0-P1.3 空置，固定输出高（准双向口不用时置高更稳）
 * ================================================================== */
void timer1_isr(void) interrupt 3
{
    unsigned char out = 0;

    if (++pwm_tick >= PWM_LEVELS)
        pwm_tick = 0;

    if (pwm_tick < duty_lf) out |= 0x40;    /* P1.6 */
    if (pwm_tick < duty_lr) out |= 0x80;    /* P1.7 */
    if (pwm_tick < duty_rf) out |= 0x20;    /* P1.5 */
    if (pwm_tick < duty_rr) out |= 0x10;    /* P1.4 */

    P1 = (unsigned char)(0x0F | out);
}

/* ==================================================================
 * 延时
 * ================================================================== */

/* 轮询 Timer0，11.0592MHz 下 1ms = 922 个计数 */
void delay_ms(unsigned int ms)
{
    while (ms >= 50)
    {
        TH0 = (unsigned char)(RELOAD_50MS >> 8);
        TL0 = (unsigned char)(RELOAD_50MS & 0xFF);
        TF0 = 0;
        TR0 = 1;
        while (!TF0)
            ;
        TR0 = 0;
        ms -= 50;
    }

    while (ms--)
    {
        TH0 = (unsigned char)(RELOAD_1MS >> 8);
        TL0 = (unsigned char)(RELOAD_1MS & 0xFF);
        TF0 = 0;
        TR0 = 1;
        while (!TF0)
            ;
        TR0 = 0;
    }

    TF0 = 0;
}

/* Trig 高脉冲：10 个 NOP = 10 个机器周期约 10.9us，加指令开销约 13us */
void trig_pulse(void)
{
    TRIG = 1;
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;
}

/* ==================================================================
 * 超声波测距
 *   返回：2~450 为厘米；DIST_FAR / DIST_BLIND / DIST_INVALID 见上
 *   距离 = t x 204 / 11059（t 为 Echo 高电平的机器周期数）
 * ================================================================== */
unsigned int measure_cm(void)
{
    unsigned int wait;
    unsigned int t;
    unsigned long cm;

    /* 上次测距的 Echo 还没结束：不重复触发 */
    if (ECHO == 1)
    {
        if (++echo_high_count >= ECHO_HIGH_LIMIT)
            return DIST_INVALID;
        return DIST_FAR;
    }
    echo_high_count = 0;

    TRIG = 0;
    _nop_();
    _nop_();
    trig_pulse();

    /* 等 Echo 上升沿，约 5ms 超时 */
    wait = 0;
    while (ECHO == 0)
    {
        if (++wait > ECHO_RISE_TIMEOUT)
            return DIST_INVALID;
    }

    TH0 = 0;
    TL0 = 0;
    TF0 = 0;
    TR0 = 1;

    /* 测高电平时长；超过 5ms（约 85cm）按无障碍处理 */
    while (ECHO == 1)
    {
        t = ((unsigned int)TH0 << 8) | TL0;
        if (t > ECHO_MAX_TICKS)
        {
            TR0 = 0;
            TF0 = 0;
            return DIST_FAR;
        }
        if (TF0)
        {
            TR0 = 0;
            TF0 = 0;
            return DIST_FAR;
        }
    }

    TR0 = 0;

    if (TF0)
    {
        TF0 = 0;
        return DIST_FAR;
    }

    t = ((unsigned int)TH0 << 8) | TL0;
    cm = (unsigned long)t * 204UL / 11059UL;

    if (cm > 450UL)
        return DIST_FAR;
    if (cm < 2UL)
        return DIST_BLIND;

    return (unsigned int)cm;
}

/* ==================================================================
 * 电机控制
 *   4 路占空比独立，便于以后单独微调每个轮子
 * ================================================================== */
void set_duty(unsigned char lf, unsigned char lr,
              unsigned char rf, unsigned char rr)
{
    duty_lf = lf;
    duty_lr = lr;
    duty_rf = rf;
    duty_rr = rr;
}

void car_forward(void)
{
    LEFT_FWD();
    RIGHT_FWD();
    set_duty(CRUISE_DUTY, CRUISE_DUTY, CRUISE_DUTY, CRUISE_DUTY);
}

/* 左转修正：黑线偏左，左轮慢、右轮快 */
void car_steer_left(void)
{
    LEFT_FWD();
    RIGHT_FWD();
    set_duty(INNER_DUTY, INNER_DUTY, OUTER_DUTY, OUTER_DUTY);
}

/* 右转修正：黑线偏右，右轮慢、左轮快 */
void car_steer_right(void)
{
    LEFT_FWD();
    RIGHT_FWD();
    set_duty(OUTER_DUTY, OUTER_DUTY, INNER_DUTY, INNER_DUTY);
}

void car_pivot_left(void)
{
    LEFT_REV();
    RIGHT_FWD();
    set_duty(PIVOT_DUTY, PIVOT_DUTY, PIVOT_DUTY, PIVOT_DUTY);
}

void car_pivot_right(void)
{
    LEFT_FWD();
    RIGHT_REV();
    set_duty(PIVOT_DUTY, PIVOT_DUTY, PIVOT_DUTY, PIVOT_DUTY);
}

/* 短接制动：IN 全 0 + 使能全开，电机两端被低阻短接 */
void car_stop(void)
{
    LEFT_STOP();
    RIGHT_STOP();
    set_duty(PWM_MAX, PWM_MAX, PWM_MAX, PWM_MAX);
}

/* 滑行：使能全关，电机两端高阻 */
void car_coast(void)
{
    set_duty(0, 0, 0, 0);
}

/* ==================================================================
 * 循迹
 *   返回 0：仍在线上（已给出动作）；1：两个探头都脱线
 *   last_dir：1 = 黑线在左，2 = 黑线在右，供找线时优先方向
 * ================================================================== */
unsigned char line_follow_step(unsigned char *last_dir)
{
    unsigned char left_on;
    unsigned char right_on;

    left_on  = (LEFT_SENSOR  == LINE_BLACK_LEVEL) ? 1 : 0;
    right_on = (RIGHT_SENSOR == LINE_BLACK_LEVEL) ? 1 : 0;

    if (left_on && right_on)
    {
        car_forward();
        return 0;
    }
    else if (left_on)
    {
        *last_dir = 1;
        car_steer_left();
        return 0;
    }
    else if (right_on)
    {
        *last_dir = 2;
        car_steer_right();
        return 0;
    }

    car_stop();
    return 1;
}

/* ==================================================================
 * 主程序
 * ================================================================== */
void main(void)
{
    unsigned int d;                     /* 最近一次测距的结果：2~450 是厘米，否则是下面三种特殊码 */
    unsigned char loop_count = 0;       /* 主循环分频计数 0~19：为 0 的那一轮去测距，20 轮 = 100ms */

    unsigned char state = STATE_FOLLOW; /* 状态机当前在哪个状态：FOLLOW/SEARCH/OBSTACLE/LOST/FAULT */
    unsigned char clear_count = 0;      /* 障碍移开确认：连续 3 次读到 >20cm 才允许退出 OBSTACLE */
    unsigned char fault_count = 0;      /* 进故障确认：连续 3 次 DIST_INVALID 才认定模块坏了 */
    unsigned char fault_ok_count = 0;   /* 出故障确认：连续 3 次有效读数才允许离开 FAULT */

    unsigned char last_dir = 0;         /* 最近一次单探头压线时黑线在哪边：1=左 2=右 0=从没见过 */
    unsigned char search_dir = 1;       /* 本次找线决定往哪转：从 last_dir 继承，未见过线时默认向左 */
    unsigned int  search_walk = 0;      /* 找线第一阶段：还要直行多少 ms，减到 0 改为原地转 */
    unsigned int  search_turn = 0;      /* 找线第二阶段：还要原地转多少 ms，减到 0 判定找线失败 */
    unsigned char resume_state = STATE_FOLLOW; /* 遇障/故障之前是什么状态，事情过去后回到这里 */

    /* ===== 上电初始化：先把一切输出关干净，再启动外设 =====
     * P1 = 0x0F：高 4 位（P1.4~P1.7，PWM 使能脚）清零 = 4 路使能全关；
     *            低 4 位置 1 = 悬空脚处于弱上拉，更抗干扰。
     * LEFT/RIGHT_STOP()：8 个方向脚全 0 = L298N 输出短接制动，
     *            但使能是关的，电机两端高阻，车只是"不会动"。
     * car_coast()：占空比全 0，双保险。
     * 顺序不能反：若先 pwm_init() 再关输出，Timer1 中断一跑起来，
     * duty_* 还是上电的随机值，P1 会输出随机波形，轮子可能猛抖一下。
     */
    P1 = 0x0F;
    LEFT_STOP();
    RIGHT_STOP();
    car_coast();

    timer0_init();      /* Timer0：延时和测距用，只数数、不开中断 */
    pwm_init();         /* Timer1：从这里开始 5kHz 中断，PWM 自动输出 */
    delay_ms(1000);     /* 等 L298N 和超声波模块上电稳定，别急着跑 */

    while (1)
    {
        /* ==========================================================
         * 第一段：测距 + 障碍/故障判断
         * loop_count 是分频器：20 轮里只有第 0 轮真正测距，其余 19 轮
         * 直接跳过这段。所以这里面的代码每约 100ms 才执行一次。
         * ========================================================== */
        if (loop_count == 0)
        {
            d = measure_cm();   /* 返回 2~450cm，或 DIST_FAR/BLIND/INVALID */

            /* --- 情况 1：测量失败 = d 是 DIST_INVALID = 模块"没回答" ---
             * measure_cm 只在两种时候返回 DIST_INVALID：
             *   a) 发了 Trig 之后约 10ms 内 Echo 一直没拉高——线断了、
             *      模块没供电、或模块内部坏了，总之问它它不吭声；
             *   b) Echo 一直挂在高电平，连续 3 次测距都这样（模块卡死）。
             * 注意和 DIST_FAR 区分：FAR 是"答了：前方没东西"，
             * 这里是"根本没答"。没答必须停车，因为不知道前方啥情况。
             */
            if (d == DIST_INVALID)
            {
                fault_ok_count = 0;   /* "连续 3 次有效"的恢复计数清零 */

                /*
                 * 短路求值：state 已经是 FAULT 时，右边的 ++fault_count
                 * 根本不执行——既冻结累加，也避免恢复后条件立即再成立。
                 * 连续 3 次失败（约 300ms）才认定模块真坏了。
                 */
                if (state != STATE_FAULT && ++fault_count >= FAULT_CONFIRM)
                {
                    resume_state = state;   /* 记住坏之前在哪，修好后回去 */
                    car_stop();
                    state = STATE_FAULT;    /* 停车，等模块自愈 */
                }
            }
            else
            {
                /* --- 测量成功：故障计数立刻清零（偶发一次失败不算数）--- */
                fault_count = 0;

                if (state == STATE_FAULT)
                {
                    /* 情况 2：车正处在 STATE_FAULT 状态（情况 1 里连续 3 次
                     * 失败时进去的），而这回测量居然成功了——要决定能不能
                     * "出院"：连续 3 次有效读数（fault_ok_count 数满 3）才敢
                     * 回到故障前的状态，防止模块时好时坏把车放出去又趴窝。 */
                    if (++fault_ok_count >= FAULT_RECOVER)
                    {
                        fault_ok_count = 0;

                        /* 恢复前先看这次的读数：障碍就在眼前就直接进
                         * OBSTACLE，别回到行驶状态后又立刻停下 */
                        if (d == DIST_BLIND ||
                            (d != DIST_FAR && d <= STOP_DISTANCE_CM))
                            state = STATE_OBSTACLE;
                        else
                            state = resume_state;
                    }
                }
                else if (d == DIST_BLIND ||
                         (d != DIST_FAR && d <= STOP_DISTANCE_CM))
                {
                    /*
                     * 情况 3：正常行驶中，前方 15cm 内出现障碍（或进盲区）。
                     * d != DIST_FAR 是防御性写法：DIST_FAR = 65534，
                     * 显式排除，读的人不用心算"65534 > 15"。
                     */
                    if (state != STATE_OBSTACLE)
                        resume_state = state;   /* 只在头一次进来时记，别覆盖 */
                    state = STATE_OBSTACLE;
                    clear_count = 0;            /* 移开确认从零开始数 */
                    car_stop();
                }
                else if (state == STATE_OBSTACLE)
                {
                    /* 情况 4：正在障碍前等待——判断移开没有 */
                    if (d == DIST_FAR || d > RESUME_DISTANCE_CM)
                    {
                        /* 连续 3 次读到 >20cm 才走，防单次读数抖动误判。
                         * 恢复阈值(20)比停车阈值(15)大 5cm，形成回差：
                         * 障碍在 15~20cm 间晃动时，车不会反复启停 */
                        if (++clear_count >= RESUME_CONFIRM)
                        {
                            clear_count = 0;
                            state = resume_state;   /* 回到遇障前的状态 */
                        }
                    }
                    else
                    {
                        clear_count = 0;   /* 中途又变近了？重新数 */
                    }
                }
            }
        }

        /* ==========================================================
         * 第二段：按当前状态执行动作，每轮（约 5ms）都执行。
         * 分工：测距段只负责"改 state"，这里只负责"按 state 做事"。
         * ========================================================== */
        switch (state)
        {
        case STATE_FOLLOW:
            /* 正常循迹：读两个探头并在函数内部直接给出动作。
             * 返回 1 = 双探头都脱线，转入找线流程 */
            if (line_follow_step(&last_dir))
            {
                state = STATE_SEARCH;
                search_dir = (last_dir == 2) ? 2 : 1;  /* 沿最后看到线的方向找；没见过线(0)默认向左 */
                search_walk = SEARCH_WALK_MS;   /* 第一阶段：先直行 150ms */
                search_turn = SEARCH_TOTAL_MS;  /* 第二阶段：再原地转 4840ms */
            }
            break;

        case STATE_SEARCH:
            /*
             * 找线流程，每轮从上往下依次判断（四步是串行的）：
             *   ① 任一探头压到线 → 立即回 FOLLOW（直行和转动途中都在检测）；
             *   ② search_walk 没减完 → 继续直行（救黑线断口/十字路口）；
             *   ③ search_walk 用完 → 原地转，search_turn 递减；
             *   ④ search_turn 也用完 → 转满一圈多仍没找到，放弃进 LOST。
             * ②末尾的 break 保证直行阶段不会顺手去扣 search_turn，
             * 两个阶段严格串行：总时长 = 150ms + 4840ms，不是并行倒计时。
             */
            if (LEFT_SENSOR == LINE_BLACK_LEVEL ||
                RIGHT_SENSOR == LINE_BLACK_LEVEL)
            {
                state = STATE_FOLLOW;   /* ① 找到了 */
                break;
            }

            if (search_walk > 0)        /* ② 第一阶段：直行 */
            {
                car_forward();

                if (search_walk > LOOP_MS)
                    search_walk -= LOOP_MS;   /* 每轮扣 5ms */
                else
                    search_walk = 0;
                break;
            }

            if (search_turn > 0)        /* ③ 第二阶段：原地转 */
            {
                if (search_dir == 1)
                    car_pivot_left();   /* 线最后在左边 → 向左转回去找 */
                else
                    car_pivot_right();

                if (search_turn > LOOP_MS)
                    search_turn -= LOOP_MS;
                else
                    search_turn = 0;
            }
            else
            {
                car_stop();             /* ④ 放弃 */
                state = STATE_LOST;
            }
            break;

        case STATE_OBSTACLE:
            /* 障碍停车：只停，完全不看循迹探头——哪怕线就在脚下也不动，
             * 防止车在障碍物前蠕动摇摆。何时恢复由测距段的情况 4 决定。 */
            car_stop();
            break;

        case STATE_LOST:
            /* 找线失败：停车等人工把车放回线上；探头碰到线自动恢复 */
            car_stop();

            if (LEFT_SENSOR == LINE_BLACK_LEVEL ||
                RIGHT_SENSOR == LINE_BLACK_LEVEL)
            {
                state = STATE_FOLLOW;
            }
            break;

        case STATE_FAULT:
        default:
            /* 超声波故障：停车等模块恢复（恢复逻辑在测距段的情况 2） */
            car_stop();
            break;
        }

        /* ===== 节拍：每轮睡 5ms（实际 5.5~6ms，会被 Timer1 中断拖长）===== */
        delay_ms(LOOP_MS);

        if (++loop_count >= MEASURE_LOOPS)
            loop_count = 0;   /* 数满 20 归零，下一轮 loop_count==0，又该测距了 */
    }
}

/*
 * 调试顺序建议：
 * 1. 先确认黑线极性：车骑在黑线上时左右 OUT 都应为 LINE_BLACK_LEVEL，
 *    相反就把 LINE_BLACK_LEVEL 改成 0。
 * 2. 确认 25% 内侧轮能转：不能转就把 INNER_DUTY 提到 30%。
 * 3. 循迹左右摆：减小 OUTER 与 INNER 的差值；
 *    转不过弯：加大差值（OUTER 提到 45%，INNER 降到 20%）。
 * 4. 找线角度：先标定 PIVOT_90_MS（35% 占空比下原地转 90 度的时间），
 *    搜索总时长按 4 倍加 10% 余量自动算出。总是扫不到线就加大它。
 * 5. 丢线后车总往赛道外跑：减小 SEARCH_WALK_MS；
 *    黑线断口过不去：加大 SEARCH_WALK_MS。
 * 6. 停车距离：低速下预期 13~14cm 停住；偏差超过 3cm 再改
 *    BRAKE_TRAVEL_CM。
 */
