一 .结构框架

- modules:

  - animation:用于仿真数据显示与记录数据回放
    - environment:显示机器运行状况,包括机器位姿,路径,轨迹,障碍物(原始),地图点云,算法数据(动态系统,mpc,目标点,障碍物处理结果等数据)
    - cmd:控制下发速度&反馈速度(v&w)显示
    - map:地图&边界信息(png),规划路径(完整的全覆盖路径),机器位置&方向
    - sensor:机器相机布局与视野范围,模拟激光雷达信号,障碍物点云,栅格地图点云
    - space:机器三维空间姿态
    - curve:曲面速度规划—遇障速度规划(normal+distance),跟踪横纵误差规划

  - control

    - 基础算法模块
      - 控制基础参数
      - 避障:机器轮廓处理(虚拟轮廓(bounding volume),jacobian等),点云处理(采样,聚类,合并,插值),速度规划,避障算法(动态系统,dwa等)
      - 跟踪(横向控制):并联PID(直线),modify几何控制法(pursuit,stanley),mpc
      - 速度规划(纵向):路径平滑(qp),路径分类提取(curve类型,危险边界),速度规划(QP or状态机)

    - 功能模块

      跟踪功能:不同跟踪算法切换(直线,曲线 or pose控制),控制内部线程维护

      避障功能:数据更新→障碍物处理→障碍物矢量→速度规划→避障算法选择

      基础动作:按距离直行,划弧,原地旋转,停机等

    - 接口模块
      - 控制内部线程维护,功能状态机

  - csvplot
    - 采集数据解析
    - 调度ainimation模块进行显示

  - datacenter:各模块内部数据交互,set函数使用互斥锁,get函数使用读写锁

  - environment:仿真环境设置(障碍)
    - 环境更新:障碍物位姿更新
    - 动静障碍物生成,不同形状障碍物生成,障碍物噪声设置

  - gridmap:局部障碍物栅格地图
    - 局部范围内的障碍物点云提取,传感器盲区障碍物填充

  - logger
    - csv文件操作,数据记录
    - 多数据分类(路径,环境,地图,算法数据,时间等),格式整理,多线程数据存储

  - planning:
    - 简单路径生成:直线+曲线
    - 路径平滑(qp等)

  - sensor

    模拟激光雷达,虚拟点云识别(camera frustum,极坐标射线,障碍物交点,最近交点)

  - vehicle:物理参数
    - unicycle and bycicle(state space构造)
    - configuration:几何尺寸→平面,线框,轮(主动&从动),相机视锥

    shell script:

    

- shell script
  - build.sh:程序编译(compile,make,reset,format)
  - sim_start.h:启动仿真,启动数据回放
  - csv_log.sh:SSH远程机器链接(sshpass自动密码输入),内部文件打包拉取,多终端数据回放
  - transfer.sh:SSH远程,多终端程序传输&校验,进程启动

- src
  - main程序:主要进程→机器迭代线程+数据记录线程x2+动画线程+传感器线程+控制线程+环境线程+地图更新线程

- utilities

  basic_algorithem:dbscan,interpolation

  common_port:通用数据接口

  facilities:单例,时间获取封装

- tools

  math_tools.h:数学基础运算功能函数



二:仿真&数据回放功能窗口演示

- 仿真平台
  - environment
  - cmd
  - map
  - sensor
  - space
  - curve

- 数据回放
  - 多窗口显示
  - 多功能演示
  - 支持键盘交互

三.算法细节

- 纵向控制

  曲线提取(picture)

  qp路径平滑(矩阵截图)

  qp速度规划(基本原理简介)

  曲面速度规划(横纵误差规划)[https://www.desmos.com/3d/ivdwriqikl?lang=zh-CN](https://www.desmos.com/3d/ivdwriqikl?lang=zh-CN)

- 跟踪算法

  - 直线跟踪+PP(数据回放)

  - modify 纯跟踪,stanley(gif)

  - mpc(gif)
    - 基本组成
    - 建模过程

- 避障功能(gif)
  - 障碍物处理流程:点云角度分区采样→相限合并→聚类→插值→虚拟障碍物
  - 虚拟障碍物相关信息计算:normal,tangent,distance
  - 线速度规划(gif)
  - dwa:已不再使用(gif)

  -   动态系统算法
    - 特征向量矩阵计算(tangent,reference vector)
    - stretch矩阵(gamma distance,lambda params)
    - 多障碍物权重计算(distance,向量点乘[方向])
    - 各障碍物modulate计算:modulated matrix * ref_v
    - 多障碍物space-k向量合成,多障碍物权重叠加


