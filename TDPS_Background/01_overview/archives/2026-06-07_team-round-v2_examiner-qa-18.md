# TDPS 团队轮 v2 - Examiner 细节质询 Q&A 总表(18/18)存档

> 存档时间 2026-06-07 04:2X。固件 60117ff(F6)+F8(7d68f00)裁决引用。
> 团队:LogAnalyst/PostS/RunArch/Examiner/Skeptic,leader 统一裁决。
> 冲突处以 leader 裁决为准(见 PID_TUNING_LOG.md 03:55/04:05/04:15 条目)。
> ⚠ 注:A#2(S1c)条目保留 Examiner 原始推演;leader 终裁=**驳回改码**(冻结期 PID:518 corr=0 强制走直,min_inner 只抬不压不咬合;真残留在出口侧,已有 R5 软启动+滞回退出覆盖;S 区 jc 通胀下"上升沿清 deep"会中途掉 pivot)。首跑三方框采 junc=/deep= 同帧复核。

## 先行确认两条

**① Q7(02:15 半清)= 结案**。[⚠04:50 订正(Skeptic 二审,leader 验证):原措辞"全部 statics 每 tick 清"过度——清单 12 项**遗漏 g_line_lost_ticks**(:442 return 早于其全部清零点),停车后冻结残留(第3组实测 lost=266 钉屏≥6s);已 F11 补清一行。Q7 主结论(无运行态半清、02:15=运行中真实复位)不变。] 结论:**无"必须补清"项**。证据:PID 侧 statics(g_reacq_grace/g_deep_turn_mode/g_last_edge_side/g_a2_flips/g_a2_episode_cool/g_line_lost_ticks/g_reacq_run)全在 PID_Control_Update 的 `if(!is_racing)` 块(PID_Controller.c:430-438)每 tick 清零;K2→StopRun 置 is_racing=0→下一 tick 该块先跑→K1 再置 1 前已清。故怀疑的"K1/K3 半清不一致"对 PID 侧 statics 不成立(g_reacq_grace 尤其不是残留)。自答+RunArch 独立同结论=双人交叉验证。K3 早先漏清的 g_u3_deep_run 已 F8 补清。

**② g_stall_boost 最终口径**:
- 结构真实:main.c 全局,只在 StopRun(main.c:507-508)清,不归 PID `!is_racing` 块管。丢线自停走 PID_Controller.c:605-608 `is_racing=0; return` 不调 StopRun;且 PID 阈 375tick(PID:405)<主环 500tick(main.c:500),PID 自停先触发→主环 StopRun(main.c:890)可能被先到的 is_racing=0 抢掉。→ 丢线自停后 boost 是跨轮真残留。
- 后果无害:三层防御——衰减(main.c:1341 !is_racing -8/tick)、main.c:887-890 1s 路径迟到补 StopRun、F6a launch_grace 发车前 500ms 封 100。
- **K1 清属可选整洁化(非必修)**。不写成"全部 statics 都被 PID 块清"(boost 例外),不升级为必修缺陷。

---

## A. 未决/留观(按威胁排序,标闭环数据)

**#1 S4(a)/FINISH-110**(Skeptic+Examiner;leader 终裁)=**留观-需实测(非必修)**
唯一几何推导未实测的里程门。FINISH_FROM_S2_CNT=110(main.c:286)由几何 249cm/2.47 凑;运行用带符号均值,pivot 区外轮半速少计~2×(非塌缩;内轮 coast 不反转,min_inner=0 钳负摄动 PID:756)→110 绝对行程偏晚触发→可能拖到红区边缘。T2=200/T3=240 按实测 sm_dcnt 同币种整定已自洽。
闭环:首跑雷达段后用 lt=4(re-arm)起算到终点实测 Δ 回填 110。

**#2 S1(c)/P2 三方框 T 字冻结继承 deep**(三人)=**未决-需实测**[leader 终裁:改码提案驳回,见文头注]
deep 判据 else if(!is_junction)(PID:734)→路口冻结时 deep 保持入口前值。三方框 T 字=单段宽黑(BlackPoint_Finder.c:315)必触冻结(U 弯双腿 run_count==2 被排除)。Examiner 原推演"斜入+入口已 deep→内轮停转→绕内轮冲支线"——经 leader 对码驳回(冻结期 corr=0,差速块两轮同速)。真残留=出口侧带 deep 旗对中等误差给满差速,已有覆盖。
闭环:首跑三方框录像 + 遥测 deep= 在 junc=1 期间与出口侧行为。

**#3 S4(b)/A3 雷达 never-arm→冲场**(Skeptic+RunArch)=**未决-设计缺口(落码缓议)**
#error(main.c:290)只保证"开 S2_REARM 则兜底可用",不保证雷达运行期触发。无箱/里程没到/箱前没深丢线→RD 不 arm→g_s2_active=0(仅 main.c:1199 置)→里程终点兜底(main.c:1253)死→只剩 0x30。RD_FAIL→StopRun 无害(已停)。建议加不依赖 g_s2_active 的纯总程兜底(leader:设计采纳,阈值等全程实测后落码;过渡协议=雷达没 arm 必 K2 手停)。
闭环:确认 0x30 部署;首跑雷达是否 arm。

**#4 P3 顶圆缺口 50cm**(PostS)=**未决-需实测底图**
freeze 24cm+lostline 36cm=60>50 仅当顶连线先触冻结;纯丢线 36<50→缺口内 375 自停(PID:605)。四圆出口≈371cnt 临近 RD_ZONE_MIN_CNT=380(main.c:230)→可能误当雷达箱。
闭环:首跑顶连线处 junc= 是否=1 + lost= 峰值 + 四圆出口 Δcnt vs 380。

**#5 S2(b)→(a) sm 早释放污染/死锁**(Skeptic)=**未决-需实测**
S①出口 273cm 长直→稳线窗 500ms 早释放→g_sm_cnt_base 固化污染 RD 门。方块阵反复重置稳线窗(main.c:1051)→退守 T3 240cnt 慢爬(非挂科)。
闭环:首跑 S 出口 sm= 释放点 + rs=(释放源,F8)+ 方块阵段 sm= 是否钉住。

**#6 S5 C4-blind 盲冲区**(Skeptic)=**未决-需实测**
125~375tick 全增益冲,750ms@25cps≈45cm;误判侧(阴影 16tick)≈1.9cm。建议 375 随速缩放/丢线降速提前(现 lost>10)。
闭环:各段道宽 vs 45cm;出界录像。

**#7 P4 入弯跑宽=锁存晚整弧裸跑**(PostS)=**已解机理/需实测**
Δ140cnt(U3 Δ60 vs T2 Δ200)=晚锁 2-3 弧≫降速收敛。03:30 jc 漏检→T2@Δ201→前 2-3 弧裸跑。U3(F6b)来接管。
闭环:首跑 lt= 锁存源 + S 第一弧起点车速。

---

## B. 已解(机理闭合/裁决落码)

**S3 U3 误锁现值安全**(Skeptic 量化)=**已解(红线)**
U尾30° Δ≈2~5cnt(几何4.4/pivot2.9/带符号2.3)≪60。OLED 实锤 03:30 走 SM FORCED LATCH(T2,Δ201),U3 未在 U 出口开火。红线:**SM_DEEP_MIN_CNT=60 严禁下调**(防 U 尾唯一闸)。

**L4 + A4(K3 g_u3_deep_run)观测性**(LogAnalyst)=**已解-F8 落码(7d68f00)**
遥测新增 lt=(0未锁/1jc/2U3/3T2/4re-arm)+ rs=(0未放/1主锚/2T3/3后备),K1/K3 清;K3 已补清 g_u3_deep_run。原"sm=1 不可分辨扣扳机源 / DEEP LATCH 不可溯源 / K3 漏清"全消解。

**L1 + Skeptic T3 二段弹射**(LogAnalyst)=**已解-驳回**
main.c:1366-67 对 g_stall_boost 本体直接赋值(无影子变量),先 +4(1338)后钳 100(1364)每帧吃超出量→解锁瞬间本体=100,之后 +4/tick 平滑爬,无台阶。"只钳输出不钳累加器→500ms 二段弹射"主张不成立。

**T1 根因**(Skeptic+Examiner+A1;leader 终裁)=**已解-非必修(110 留观)**
里程口径=带符号均值,pivot 外轮半速少计~2×(非塌缩)。T2/T3 同币种整定自洽,唯 FINISH 110 留观(见 #1)。"必修"→"高危待标定"。

**A1**(自答+RunArch 日志锤)=**已解(待 P0a)**
三源链互斥(main.c:967/975/990)写一次。03:30 走 T2(log:947)。锚平移危害落 RD-arm 时机(与丢线门 AND→不提前只延后)。

**A2**(自答+RunArch)=**已解**
T2 正常路况死码(U3 抢先)但非冗余=deep 连续性互补;03:30 deep 被打断→T2 兜底。两者都留。

**A4(g_stall_boost)**=**已解-非必修**(口径见上②)。

**A5**(自答+RunArch 计数订正)=**已解(未来雷)**
sm_done 3 置位点(1015/1046/1070)非 4。navseg 现无消费者→无害;T3 误强释提前推 navseg=未来雷(路线2 前补 done_reason)。

**L2-L3**(LogAnalyst)=**已解**
L2 speed>12 衰减(-8)主导;L3 vel_t0 static K1 不重置→窗 free-run+首窗稀释→grace=500=2×最坏窗。单位陷阱:speed_* 是窗口 Δ 非 cps(1cnt≈4cps)。

**P1**(PostS 订正)=**已解**
U3 评估在 sm 锁前,deep 滞回 1.9/1.5(PID:728)非 1.7/1.2(1.7/1.2 是 sm 域);跨弧过零强制退出→25tick 单弧内凑满(390tick≫25)。

**P2**(PostS)=**已解**
U 弯靠 deep pivot 过非 jc 冻结(BlackPoint_Finder.c:315 run_count==1 排除双腿);内轮停转 R6.5<20 留裕度;原问题是外甩 R37>20。03:04/03:30 干净=U1/U2 首验。

**S1(a)(b)**(Skeptic)=**已解**
冻结期(≤400ms,BPF:44)里程门照走、踢腿正常。

**D5 ESP32_Tick**(RunArch;leader 闭案)=**已解-闭案**
调用点在 main.c:869(#if 块内),RunArch 漏看,非代码漏。

---

## C. 落地清单

**改码(leader 裁决后状态)**:
① junction 上升沿强制 g_deep_turn_mode=0 → **驳回**(见文头注)。
② 不依赖 g_s2_active 的纯总程里程兜底 → **设计采纳,落码缓议**(阈值等全程实测;过渡=雷达没 arm 必 K2)。
③ lt=/rs= 遥测 + K3 清 g_u3_deep_run → **已 F8 落码**。

**标定(首跑后)**:FINISH 110 用 lt=4 起算实测 Δ 回填(唯一留观门)。

**首跑必采(F8 字段就位)**:
- lt= → 验 U3 接管(P4/P1)、U 尾不误锁(S3)、T2 兜底复现(A1/A2)
- rs= → 验 sm 出口源(S2b)
- el/er 跨 u=1 / sm=1 / lt=4 沿的 Δ → 重建 U尾Δ/S②Δ/FINISH 标定基准
- 雷达是否 arm(没 arm 必 K2 手停防冲场)
- 三方框 + 顶连线处 junc= / deep= / lost=

**红线**:SM_DEEP_MIN_CNT=60 严禁下调。

## 关键文件行号
- main.c:三源锁存 967-998 / 终点兜底 1247-1259 / 遥测帧 1571-1591(含 lt/rs)/ StopRun 503-508 / 主环丢线停 887-890 / ESP32_Tick 869 / K1K3 1450-1508
- PID_Controller.c:deep/min_inner 727-784(负摄动钳 756)/ 统一清零 425-442 / 丢线自停 605-608
- BlackPoint_Finder.c:全黑 283 / 冻结 318 / 选段 341 / junction 判据 315
- ABEncoder.c:带符号累加 53-58
