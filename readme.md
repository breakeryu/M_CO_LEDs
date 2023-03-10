

# 一、调试LED

CIA 303-3标准规定了用于指示的LED，它反映了CANopen设备的状态。可以使用绿色和红色led或双色led。

- CANopen 绿色led -运行led:
  - 闪烁(flickering):LSS配置状态处于活动状态
  - 闪烁(blinking):设备处于NMT预运行状态
  - 单闪:设备处于NMT停止状态 
  - 三闪:软件下载正在设备中运行 
  - 亮:设备处于NMT操作状态 
- CANopen 红色led -错误led:
  - 关:无错误 
  - 闪烁(flickering):LSS节点id未配置，CANopen未初始化
  - 闪烁(blinking):无效配置，一般错误 
  - 单闪:是否达到警告限值 
  - 双闪:心跳消费者-远程监控节点中的错误
  - 三闪:同步消息接收超时 
  - 四次闪烁:在事件计时器超时之前，未收到PDO 
  - 开:CAN总线关闭

> Q1：[What is Network management (NMT)?](https://www.can-cia.org/can-knowledge/canopen/network-management/)
>
> All CANopen devices must support the CANopen network management (NMT) slave state machine. The NMT state machine defines the communication behavior of a CANopen device. The CANopen NMT state machine consists of an Initialization state, a Pre-operational state, an Operational state, and a Stopped state. After power-on or reset, the device enters the Initialization state.
>
> PS:
>
> CANopen device can be in one of the [CO_NMT_internalState_t](https://canopennode.github.io/CANopenSocket/group__CO__NMT__Heartbeat.html#ga1e8c2a6c0fd4a33183503d25a7c6d744)
>
> - Initializing. It is active before CANopen is initialized.
> - Pre-operational. All CANopen objects are active, except PDOs.
> - Operational. Process data objects (PDOs) are active too.
> - Stopped. Only Heartbeat producer and NMT consumer are active.
>
> NMT master can change the internal state of the devices by sending [CO_NMT_command_t](https://canopennode.github.io/CANopenSocket/group__CO__NMT__Heartbeat.html#gac396242e2e12ef6b0b22ff48636bc4eb).

> Q2: [What is Layer Setting Services (LSS)?](https://www.can-cia.org/can-knowledge/canopen/cia305/)
>
> LSS distinguishes between an LSS manager (typically residing in the host controller) and the LSS servers. LSS enables the LSS manager to modify the LSS server’s CANopen node-ID and to switch the entire network from one data rate to another. LSS utilizes exactly two CAN frames. The CAN data frame 7E5h carries the command from the LSS manager to one or several LSS servers. The CAN frame 7E4h is used to provide the response(s) to the LSS manager. LSS is specified in the document CiA 305.

> Q3: [What is Process data object (PDO)?](https://www.can-cia.org/can-knowledge/canopen/pdo-protocol/)
>
> Process data objects (PDOs) are used in CANopen for broadcasting high-priority control and status information. A PDO consists of a single CAN frame and communicates up to 8 byte of pure application data. Device designers have to evaluate the amount of process data that the device needs to receive and transmit. Based on the result of this evaluation process, they have to provide the related amount of receive and transmit PDOs within the device.

# 二、构建自己的LED指示器

《CANopen Interface Technical Reference Manual for Encoder Products Company's Absolute Encoders》第4页描述了该编码器设备的LED指示器的规定。

![image-20230306203148720](https://img2023.cnblogs.com/blog/1423856/202303/1423856-20230306203151823-31669465.png)

![image-20230306203203119](https://img2023.cnblogs.com/blog/1423856/202303/1423856-20230306203204335-749755377.png)

<img src="https://img2023.cnblogs.com/blog/1423856/202303/1423856-20230306203213889-1852718379.png" alt="image-20230306203212647" style="zoom: 75%;" />

我们仿照该项目，写自己的LED指示器驱动代码。

## 功能定义

🟢绿色LED

| led状态          | 指示功能                       | 备注                       |
| ---------------- | ------------------------------ | -------------------------- |
| 闪烁(flickering) | 系统配置状态                   |                            |
| 闪烁(blinking)   |                                | 如果需要，可以自定义该功能 |
| 单闪             | 系统管理状态机处于停止工作状态 |                            |
| 三闪             | 系统管理状态机处于预工作状态   |                            |
| 常亮             | 系统管理状态机处于工作状态     |                            |

🔴红色LED

| led状态          | 指示功能                                    | 备注                   |
| ---------------- | ------------------------------------------- | ---------------------- |
| 关               | 无错误                                      |                        |
| 闪烁(flickering) | 系统重要参数未初始化，致命错误，优先级第2位 |                        |
| 闪烁(blinking)   | 其他错误，优先级最后                        |                        |
| 单闪             | 致命错误5，优先级第6位                      | 如果需要，自定义该功能 |
| 双闪             | 致命错误4，优先级第5位                      | 如果需要，自定义该功能 |
| 三闪             | 致命错误3，优先级第4位                      | 如果需要，自定义该功能 |
| 四次闪烁         | 致命错误2，优先级第3位                      | 如果需要，自定义该功能 |
| 常亮             | 致命错误1，优先级第1位                      | 如果需要，自定义该功能 |

