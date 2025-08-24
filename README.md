<!-- # Marvelous-Literature-Review -->
<h1 align="center">Marvelous Literature Review</h1>

# Contents - 目录
<nav>
<ul>
  <li><a href="#Mathmatical_Theory"> 1. Mathmatical Theory - 数学理论</a></li>
  <li><a href="#Signal_Processing"> 2. Signal Processing - 信号处理</a></li>
  <ul>
    <li><a href="#Kalman_Filter"> 2.1 Kalman Filter - 卡尔曼滤波</a></li>
  </ul>
  <li><a href="#Optimal_Control"> 3. Optimal Control - 最优化控制</a></li>
  <ul>
    <li><a href="#Model_Predictive_Control"> 3.1 Model Predictive Control - 模型预测控制</a></li>
  </ul>
  <li><a href="#Machine_Learning"> 4. Machine Learning and related - 机器学习及相关</a></li>
  <ul>
    <li><a href="#Supervised_Learning"> 4.1 Supervised Learning - 监督学习</a></li>
    <li><a href="#Unsupervised_Learning"> 4.2 Unsupervised Learning - 无监督学习</a></li>
    <li><a href="#Reinforcement_Learning"> 4.3 Reinforcement Learning - 强化学习</a></li>
    <ul>
      <li><a href="#reinforcement_learning"> 4.3.1 Reinforcement Learning - 强化学习</a></li>
      <li><a href="#Deep_Reinforcement_Learning"> 4.3.2 Deep Reinforcement Learning - 深度强化学习</a></li>
    </ul>
    <li><a href="#Characterization/Modeling_Techniques"> 4.4 Characterization/Modeling Techniques - 表征/建模技术</a></li>
    <ul>
      <li><a href="#Deep_Learning"> 4.4.1 Deep Learning - 深度学习</a></li>
    </ul>
  </ul>
  <li><a href="#Computer_Vision"> 5. Computer Vision - 计算机视觉</a></li>
  <li><a href="#Large_Language_Model"> 6. Large Language Model - 大语言模型</a></li>
  <li><a href="#Vision_Language_Model/Vision_Language_Action"> 7. Vision Language Model/Vision Language Action Model - 视觉-语言模型/视觉-语言-动作模型</a></li>
  <li><a href="#Agent"> 8. Agent - 智能体</a></li>
  <li><a href="#Medical_Robot"> 9. Medical Robot(Surgical robot) - 医疗和手术相关机器人</a></li>
</ul>
</nav>


## 1. Mathmatical Theory - 数学理论 <a id="Mathmatical_Theory"></a>














## 2. Signal Processing - 信号处理 <a id="Signal_Processing"></a>

### 2.1 Kalman Filter - 卡尔曼滤波 <a id="Kalman_Filter"></a>














## 3. Optimal Control - 最优化控制 <a id="Optimal_Control"></a>

### 3.1 Model Predictive Control - 模型预测控制 <a id="Model_Predictive_Control"></a>

- Implementation of Nonlinear Model Predictive Path-Following Control for an Industrial Robot 
*T. Faulwasser, T. Weber, P. Zometa and R. Findeisen; IEEE Transactions on Control Systems Technology 2017 Vol. 25 Issue 4 Pages 1505-1511; DOI: 10.1109/tcst.2016.2601624*

  **想解决的问题**

  本文旨在解决在工业机器人中，实现路径跟踪控制的实时、非线性、模型预测控制（NMPC）问题。具体而言，作者关注的是在存在输入和状态约束的情况下，机器人如何精确地跟随预定的几何路径，同时满足实时性的要求。这对于诸如铣削、胶合或高精度测量等需要精确路径跟踪的机器人应用非常重要。

  **具体做法**

  *算法部分：*

  1. **路径跟踪问题的重新表述：**
    - 将传统的轨迹跟踪问题重新表述为路径跟踪问题，其中路径仅由几何参考定义，而速度并未预先指定。这意味着控制器需要计算不仅仅是跟随路径的控制输入，还需要动态调整路径参数的演化，以确保精确的路径跟踪。
    - 引入了一个虚拟状态变量 \( \theta \)（路径参数），并通过一个额外的微分方程（称为定时律）来描述其时间演化：
      \[
      \theta^{(r+1)} = v
      \]
      其中，\( r \) 是定时律的阶数，\( v \) 是虚拟输入。

  2. **非线性模型预测路径跟踪控制器（MPFC）的设计：**
    - **增广系统建模：** 将原始系统与定时律结合，形成一个增广系统：
      \[
      \begin{cases}
      \dot{x} = f(x, u) \\
      \dot{z} = l(z, v) \\
      e = h(x) - p(z_1)
      \end{cases}
      \]
      其中，\( z \) 是包含路径参数及其导数的虚拟状态向量，\( e \) 是路径跟踪误差。
    - **优化问题构建：** 在每个采样时刻，求解以下优化问题：
      \[
      \min_{u(\cdot), v(\cdot)} \int_{t_k}^{t_k + T} F(e(\tau), z(\tau), u(\tau), v(\tau)) d\tau
      \]
      **约束条件：**
      - 系统动力学约束
      - 输入和状态约束
      - 路径跟踪误差定义
    - **成本函数设计：** 采用二次型成本函数，包含路径误差和控制输入的加权项：
      \[
      F(e, z, u, v) = \|e\|_Q^2 + \|(u, v)\|_R^2
      \]
      权重矩阵 \( Q \) 和 \( R \) 被设计为对路径跟踪误差和控制努力的权衡。

  *系统（设计）部分：*

  1. **机器人模型简化：**
    - 选择了KUKA LWR IV机器人，并只激活了关节1、2和4，其他关节固定。这简化了机器人的动力学模型。
    - 动力学模型：
      \[
      B(q)\ddot{q} + C(q, \dot{q})\dot{q} + \tau_F(\dot{q}) + g(q) = \tau
      \]
      其中，\( q \) 是关节角度向量，\( \tau \) 是关节力矩向量。
    - 为了加速计算，忽略了摩擦力矩中的库仑摩擦部分（用反正切函数近似），并假设机器人内部的重力补偿功能，可以忽略重力项 \( g(q) \)。

  2. **实时优化与控制实现：**
    - **自动代码生成：** 使用ACADO Toolkit的自动代码生成功能，生成C/C++代码，实现实时优化求解。
    - **输入参数化：** 控制输入被分段常数地参数化，采用10个等距间隔，每个间隔10毫秒，总的预测时域为100毫秒。
    - **实时迭代方案：** 在每个控制周期内，只执行一次SQP迭代，这是实时迭代方案的一部分，有助于满足实时性的要求。
    - **系统状态估计：** 通过磁阻编码器获得关节角度 \( x_1 \)，但关节速度 \( x_2 \) 需要通过对关节角度的有限差分和低通滤波来估计。

  3. **实验设置与接口：**
    - **硬件接口：** 使用KUKA LWR IV的快速研究接口（Fast Research Interface），允许最高1kHz的采样率。
    - **控制模式：** 机器人在关节特定的阻抗控制模式下运行，控制器可以叠加额外的力矩指令。
    - **软件实现：** 控制算法在外部PC工作站上运行，采用Linux操作系统和高性能CPU，确保在每个1毫秒的采样周期内完成优化求解和控制指令发送。

  **效果**

  - **实验验证：** 在KUKA LWR IV机器人上进行了两组实验：绘制三叶草形状的路径和绘制“Hello”字样的路径。
    - **三叶草路径实验：**
      - 实现了路径跟踪误差在1毫米以内的高精度路径跟踪。
      - 控制器能够根据路径的曲率动态调整速度，在直线路段加速，转弯处减速。
    - **“Hello”路径实验：**
      - 引入了外部扰动（人工施加外力），验证了控制器的鲁棒性。
      - 控制器能够在受到扰动时自动调整路径参数的速度，减慢或停止在路径上的移动，扰动消除后迅速回到路径并继续跟踪。
  - **实时性与计算效率：**
    - 在实验中，优化求解的最大时间为0.48毫秒，平均为0.24毫秒，满足1kHz的控制频率要求。
    - 证明了所提出的MPFC方案在实际机器人系统中具有实时可行性和良好的控制性能。

  **关键词**

  - 非线性模型预测控制（NMPC）， 路径跟踪控制， 实时优化， 机器人工程， KUKA LWR IV机器人， 增广系统建模， 虚拟状态与定时律， 自动代码生成， 实时迭代方案， 控制器鲁棒性

---

- Trajectory Generation for Multiagent Point-To-Point Transitions via Distributed Model Predictive Control 
*C. E. Luis and A. P. Schoellig; IEEE Robotics and Automation Letters 2019 Vol. 4 Issue 2 Pages 375-382; DOI: 10.1109/lra.2018.2890572*

  **想解决的问题**

  本文旨在解决多智能体系统（尤其是无人机编队）在进行点对点转移任务时的轨迹生成问题。具体来说，作者关注多智能体如何在有限的时间内，从初始位置安全地过渡到指定的目标位置，同时避免碰撞，并满足状态和控制输入的约束。在多智能体系统中，生成碰撞自由的轨迹是一个关键且具有挑战性的问题，特别是当参与的智能体数量较多时，计算复杂度和实时性要求使得传统的集中式方法难以适用。因此，作者希望开发一种分布式的、高效的轨迹生成算法，能够在保障安全和性能的前提下，提高计算效率，适用于大型无人机集群的实时或近实时轨迹规划。

  **具体做法**

  *算法部分：*

  1. **分布式模型预测控制（DMPC）：**

    - **同步DMPC框架：** 作者采用同步的分布式模型预测控制框架，每个智能体在每个时间步同时计算其最优控制输入序列。智能体在计算之前共享其预测的未来状态信息，这样可以在考虑邻居智能体未来运动的情况下，进行碰撞检测和避免。

    - **动态模型：** 智能体被建模为三维空间中的点质量，具有双积分器动力学，即位置和速度受加速度控制。离散化后，得到线性模型，用于预测未来的状态。

  2. **按需碰撞避免策略：**

    - **预测碰撞检测：** 每个智能体利用共享的邻居智能体的未来状态信息，在预测域内检测未来是否会发生碰撞。只在检测到可能碰撞的情况下，才在优化问题中引入碰撞避免约束。

    - **碰撞避免约束：**

      - **椭球碰撞模型：** 碰撞避免被建模为智能体之间的椭球形安全区域，考虑到无人机（如四旋翼）的空气动力学效应，特别是下洗气流的影响，椭球在垂直方向上拉长。

      - **软约束处理：** 为了避免因硬约束导致优化问题不可行，碰撞避免约束被处理为软约束。引入了松弛变量，允许约束在一定范围内被违反，但在代价函数中对违反程度进行惩罚。

  3. **优化问题的构建：**

    - **目标函数：** 优化目标包括三个部分：

      - **轨迹误差惩罚：** 尽量减小预测轨迹末端与目标位置之间的误差。

      - **控制努力惩罚：** 尽量减小控制输入（加速度）的大小，达到节省能源或平滑控制的目的。

      - **控制变化率惩罚：** 尽量减小连续控制输入之间的变化率，平滑控制动作。

    - **物理约束：** 包括加速度的最大最小值约束、位置的范围约束，以及必要时的碰撞避免约束。

    - **优化求解：** 在每个时间步，智能体构建并求解上述优化问题，得到未来一段时间的控制输入序列，但只执行第一个控制输入。

  *系统（设计）部分：*

  1. **智能体模型与参数设置：**

    - **简化的动力学模型：** 为了提高计算效率，将无人机简化为点质量模型，采用双积分器动力学。虽然简化了模型，但仍然能捕捉到无人机的基本运动特性。

    - **碰撞模型参数：** 碰撞避免中使用的椭球模型参数（如椭球的尺度因子）被设置为符合真实无人机（如Crazyflie 2.0四旋翼无人机）特性的值，以考虑实际运行中的安全距离。

  2. **算法实现与优化：**

    - **并行计算策略：** 为了进一步减少计算时间，作者将智能体分成若干个集群，在多核处理器上并行求解各个集群内智能体的优化问题。集群内的智能体顺序地求解，但整个过程是并行的。

    - **算法实现工具：** 

      - **仿真环境：** 在MATLAB中实现算法，用于仿真测试和参数调试。

      - **实验环境：** 使用C++语言实现算法，采用OOQP（Object-Oriented Quadratic Programming）库作为优化求解器，以满足实时性的要求。

  3. **实验平台与通信架构：**

    - **无人机平台：** 实验使用Crazyflie 2.0微型四旋翼无人机，具有体积小、成本低的优点，适合进行多无人机集群实验。

    - **位置跟踪与控制：** 利用顶置的运动捕捉系统实时获取无人机的位置，并通过无线电通信将位置指令和估计发送给无人机。无人机内部运行基于位置的控制器，以跟踪给定的轨迹。

    - **无线通信：** 通过无线电链接与每架无人机通信，以实现控制指令的下达和状态信息的获取。

  **效果**

  - **计算效率提升：** 相比于基于序列凸规划（SCP）的方法，所提出的DMPC算法显著减少了计算时间。在仿真中，计算时间相比集中式SCP方法减少了高达97%，相比解耦的SCP方法减少了85%。同时，算法具有更好的可扩展性，适用于更大规模的智能体集群。

  - **轨迹质量与成功率：** 

    - **轨迹最优性：** 虽然以分布式方式进行规划，DMPC算法生成的轨迹在总行驶距离上仅比集中式方法增加了微小的百分比（例如，在20个智能体的情况下，增加了不到2%），表现出接近最优的性能。

    - **成功率：** 在仿真中，即使在高密度环境和较多智能体（多达200个）情况下，DMPC算法仍能保持较高的任务完成率和碰撞避免成功率。

  - **实验验证：** 

    - **小规模实验：** 在真实环境下进行了多次实验，使用了多达25架Crazyflie 2.0无人机，验证了算法在实际物理平台上的有效性。

    - **复杂场景测试：** 实验包括从初始位置阵列转移到指定图案（如字母“DSL”）的复杂点对点转移任务，验证了算法在复杂条件下的性能。

    - **实时性：** 算法能够在合理的时间内完成轨迹规划（例如，在25架无人机的实验中，规划时间约为1.8秒），满足实际应用的需求。

  **关键词**

  - 分布式模型预测控制（DMPC）, 多智能体系统, 轨迹生成, 碰撞避免, 无人机集群, 同步算法, 按需碰撞避免, 软约束, 并行计算, 四旋翼无人机

---

- Predictive End-Effector Control of Manipulators on Moving Platforms Under Disturbance 
*J. Woolfrey, W. Lu and D. Liu; IEEE Transactions on Robotics 2021 Vol. 37 Issue 6 Pages 2210-2217; DOI: 10.1109/tro.2021.3072544*

  **想解决的问题**

  本文针对移动平台（如无人空中车辆、无人水下车辆或船舶等）上操作的机械臂在受到基座扰动时，如何实现末端执行器的精准控制。传统的反馈控制方法在处理大的外部扰动时可能不足，因此作者希望开发一种能够预测基座运动并针对扰动进行补偿的控制方法，以提高机械臂在复杂环境下的轨迹跟踪精度。

  **具体做法**

  *算法部分：*

  1. **基座运动预测：**

    - **时间序列预测模型：** 利用历史的基座状态信息（位置和姿态），采用自回归（AR）模型对基座的未来运动进行预测。模型形式为：

      \[
      y(t) = \sum_{i=1}^{p} \alpha_i y(t - i)
      \]

      其中，\( y(t) \) 表示基座在时间 \( t \) 的状态（位置或欧拉角之一），\( p \) 为模型的阶数，\( \alpha_i \) 为模型系数。模型系数在每个控制循环中通过线性最小二乘法进行优化，以适应局部条件。

  2. **预测控制器设计：**

    - **轨迹转换：** 根据预测的基座未来姿态 \( \hat{T}_I^B(t + i) \)，将期望的末端执行器轨迹从惯性坐标系转换到基座坐标系：

      \[
      \hat{T}_B^D(t + i) = \hat{T}_B^I(t + i) T_I^D(t + i)
      \]

    - **代价函数构建：** 在任务空间中构建一个二次型代价函数，包含以下两部分：

      - **轨迹跟踪误差最小化：**

        \[
        c_1 = \|\hat{e} - A u\|^2_G = (\hat{e} - A u)^T G (\hat{e} - A u)
        \]

        其中，\( \hat{e} \) 是预测的末端执行器相对于基座的位姿误差向量，\( u \) 是未来控制输入（末端执行器速度）序列，\( A \) 是将速度转换为位移的矩阵，\( G \) 是权重矩阵。

      - **控制输入平滑项：**

        \[
        c_2 = \|u_0 - B u\|^2_Q = (u_0 - B u)^T Q (u_0 - B u)
        \]

        其中，\( u_0 \) 是上一次的控制输入，\( B \) 用于计算连续控制输入之间的差异，\( Q \) 是权重矩阵。

    - **优化问题求解：** 将上述两部分结合，形成一个带线性不等式约束的二次规划（QP）问题：

      \[
      \min_{u} \quad c_1 + c_2
      \]

      **约束条件：**

      - **当前时刻的关节速度约束：**

        \[
        q_{\text{min}} \leq J(q(t))^\dagger u(t) \leq q_{\text{max}}
        \]

        其中，\( J(q(t))^\dagger \) 是当前关节位置下雅可比矩阵的伪逆，\( q_{\text{min}} \) 和 \( q_{\text{max}} \) 是关节速度的上下限。

      由于仅对当前时刻的控制输入施加了关节约束，而对未来时刻的控制动作不加约束，从而简化了优化问题，保证了QP问题的线性特性和求解效率。

  *系统（设计）部分：*

  - **机械臂系统：**

    - 采用一个六自由度的工业机器人UR3作为实验平台。机械臂安装在一个可自由移动的基座上，基座的运动由人为干扰模拟（如手动摇动基座）。
    - 在机械臂基座上固定一个带有增强现实标记的板，用于与摄像头帧之间的位姿估计。摄像头用作惯性固定帧。

  - **传感与测量：**

    - 使用摄像头和增强现实标记获取基座相对于惯性帧的姿态信息。
    - 通过实时采集基座的位置信息，利用时间序列模型进行基座运动预测。

  - **控制系统：**

    - **基座运动预测模块：** 通过AR(2)模型，对基座的位置和姿态进行未来若干时刻的预测。
    - **预测控制器：** 使用预测的基座运动，将期望的末端执行器轨迹转换到基座坐标系下。构建QP优化问题，求解最优的末端执行器速度序列，仅对当前时刻的控制输入施加关节速度约束，确保运动的可行性。

  **效果**

  - **仿真实验：**

    - 在七自由度机械臂的仿真环境中，基座运动来自实际AUV在波浪中的IMU数据。结果表明，使用基于预测的控制方法，相对于传统的PI反馈控制，轨迹跟踪误差减少了高达90%。
    - 在关节速度受限的情况下（如限制在6转/分钟），带约束的QP方法比无约束的QP方法的跟踪误差降低了16%，比反馈控制降低了75%。

  - **物理实验：**

    - 在UR3机械臂上进行了实验。将机械臂基座安装在一个可移动的木板上，通过手动移动基座模拟外部扰动。
    - 实验结果显示，基于预测的控制方法相比PI反馈控制，末端执行器的**位置跟踪误差减少了77%**。尽管由于UR3的安全机制，控制信号被限制在60%，但预测控制方法仍然显著提高了控制精度。

  - **鲁棒性测试：**

    - 在不同基座运动频率下进行了测试，结果表明，预测控制方法在一定范围内对扰动频率具有鲁棒性，跟踪误差明显低于传统的反馈控制方法。

  **关键词**

  - 预测控制, 移动机器人, 优化, 二次规划（QP）, 机器人运动控制, 时间序列预测, 基座扰动补偿, 机械臂末端控制, 移动操纵器, 跟踪误差减少

---

- A Predictive Control Method for Stabilizing a Manipulator-based UAV Landing Platform on Fluctuating Marine Surface 
*R. Xu, X. Ji, J. Hou, H. Liu and H. Qian; IEEE 2021; DOI: 10.1109/iros51168.2021.9636055*

  **想解决的问题**

  本文旨在解决在海洋环境下，受波浪扰动的无人艇（USV）上操作的机械臂如何稳定末端执行器，以协助无人机（UAV）安全、准确地着陆的问题。由于USV在波浪的作用下会产生高频、大幅度的随机运动（尤其是滚转和俯仰），这给机械臂的控制带来了挑战。传统的控制方法在这种情况下容易导致控制输出过高，无法满足关节速度和带宽的限制。作者希望开发一种能够在考虑关节约束和系统带宽的情况下，补偿USV受波浪影响产生的高频波动，从而稳定机械臂末端执行器的位置和姿态，提升无人机回收的成功率。

  **具体做法**

  *系统设计部分：*

  1. **三层次控制框架**

    - **高层控制器：在线轨迹生成器**
      - 基于修改的小波网络（Modified Wavelet Network，MWN）进行USV基座运动的实时预测，生成平滑的参考轨迹。
    - **中层控制器：模型预测控制器（MPC）**
      - 基于一阶延迟系统描述每个关节的运动学特性，考虑关节速度约束和系统带宽限制。
      - 利用MPC在关节速度层面进行控制，优化关节速度命令，使机械臂末端能够跟踪参考轨迹。
    - **低层控制器：独立关节速度控制器**
      - 对每个关节进行独立的速度控制，执行中层控制器给出的关节速度命令。

  2. **机械臂和USV模型**

    - **USV模型**
      - 主要关注USV的滚转（roll）和俯仰（pitch）运动，因为这些运动对机械臂末端的影响最大。
      - 忽略USV的横摇（heave）运动，并假设USV的航向（yaw）、纵向（surge）和横向（sway）运动可以主动控制或影响较小。
      - 引入一个中间坐标系 \( C_h \)，该坐标系与世界坐标系 \( C_w \) 平行，但跟随USV的平移和航向运动，只考虑滚转和俯仰的影响。

    - **机械臂模型**
      - 假设机械臂安装在USV上，受到USV基座扰动的影响。
      - 使用关节空间的动力学模型，并考虑到低层关节速度控制器只能实现关节速度的独立控制。
      - 将机械臂的关节动力学简化为一阶延迟系统：
        \[
        \frac{\dot{q}_i}{\dot{q}_{d_i}} = \frac{1}{1 + T_{m_i}s}
        \]
        其中，\( \dot{q}_i \) 为第 \( i \) 个关节的实际速度，\( \dot{q}_{d_i} \) 为期望速度，\( T_{m_i} \) 为关节的时间常数。

  *算法部分：*

  1. **修改的小波网络（MWN）用于USV运动预测**

    - **小波网络简介**
      - 小波网络是一种非线性回归方法，利用小波函数的时频局部化特性，对非线性和非平稳信号进行建模和预测。
      - 基本形式：
        \[
        y = \psi^T \Theta + e
        \]
        其中，\( \psi \) 是小波基函数的输出，\( \Theta \) 是权重向量，\( e \) 是噪声。

    - **MWN改进**
      - **减少锯齿和奇异性问题**
        - 在直接应用小波网络进行多步预测时，可能会出现锯齿和奇异性（预测值突然剧烈变化）的情况。
        - 通过在显著性评估过程中，优先选择与最新状态相关的基函数，减少锯齿现象。
        - 引入动态边界，限制预测输出的变化率，防止奇异性：
          \[
          y_{\text{upper}} = y(t) + \dot{y}(t) \Delta t + \frac{1}{2} \ddot{y}(t) (\Delta t)^2 + \frac{1}{6} y^{(3)}_{\text{max}} (\Delta t)^3
          \]
          \[
          y_{\text{lower}} = y(t) + \dot{y}(t) \Delta t + \frac{1}{2} \ddot{y}(t) (\Delta t)^2 - \frac{1}{6} y^{(3)}_{\text{max}} (\Delta t)^3
          \]
      - **改进正交分解算法**
        - 用Householder变换替代Gram-Schmidt算法，减少数值误差，提高计算稳定性和效率。

    - **MWN的在线训练和预测流程**
      - **数据准备：**
        - 使用滑动窗口提取USV过去 \( W \) 个时刻的运动数据 \( x(t - W + 1), \dots, x(t) \)。
      - **模型训练：**
        - 构建输入输出对，采用最小二乘法在线训练权重 \( \Theta \)，并通过显著性评估选择重要的小波基函数。
      - **多步预测：**
        - 利用训练好的MWN模型，预测未来 \( s \) 个时刻的USV滚转和俯仰角度。
        - 在预测过程中，应用动态边界约束，确保预测值的平滑性和合理性。

  2. **模型预测控制器（MPC）设计**

    - **控制模型建立：**
      - 考虑机械臂的关节速度受一阶延迟特性影响，建立离散状态空间模型：
        \[
        X(k+1) = A X(k) + B u(k)
        \]
        其中，\( X(k) \) 包含末端执行器的位置和速度，\( u(k) \) 为关节速度控制输入。

    - **预测模型的线性化处理：**
      - 由于雅可比矩阵 \( J(q) \) 随关节位置变化，为简化计算，假设在预测时域内雅可比矩阵保持不变（在每个控制时刻更新）。
      - 将末端执行器的运动预测与关节速度控制联系起来，构建预测模型。

    - **优化目标和约束条件：**
      - **目标函数：**
        - 最小化末端执行器在预测时域内的轨迹跟踪误差和控制增量：
          \[
          \min_{\Delta u} \sum_{k=0}^{N_p} \| X(k|t) - X_d(k) \|_Q^2 + \sum_{k=0}^{N_c} \| \Delta u(k|t) \|_R^2
          \]
        - \( X_d(k) \) 为参考轨迹，由MWN预测的USV运动生成，结合期望的末端执行器姿态。
      - **约束条件：**
        - 关节速度和速度增量的约束：
          \[
          u_{\min} \leq u(k|t) \leq u_{\max}
          \]
          \[
          \Delta u_{\min} \leq \Delta u(k|t) \leq \Delta u_{\max}
          \]
        - 末端执行器工作空间约束，确保在机械臂的灵活工作空间内：
          \[
          X_{\min} \leq X(k|t) \leq X_{\max}
          \]
      - **求解方法：**
        - 将优化问题转化为二次规划（QP）问题，利用快速QP求解器在实际控制循环中实时求解。

  3. **轨迹生成和控制流程**

    - **参考轨迹生成：**
      - 根据MWN预测的USV滚转和俯仰角度，计算末端执行器在基座坐标系下的期望轨迹 \( x_d \)：
        \[
        x_d(k) = R(\hat{\alpha}(t+k), \hat{\beta}(t+k)) x_{\text{desired}}
        \]
        其中，\( R \) 为由预测的滚转 \( \hat{\alpha} \) 和俯仰 \( \hat{\beta} \) 角度组成的旋转矩阵，\( x_{\text{desired}} \) 为在理想情况下末端执行器的期望位置和姿态。
    - **控制器执行流程：**
      - **高层控制器：** 使用MWN进行USV运动预测，生成参考轨迹。
      - **中层控制器：** 利用MPC根据参考轨迹计算关节速度命令，考虑关节速度和增量约束。
      - **低层控制器：** 执行关节速度命令，控制机械臂的实际运动。

  **效果**

  - **仿真实验：**
    - 使用从真实USV收集的海上运动数据，对MWN预测的精度和轨迹的平滑性进行了验证。
    - MWN在10步预测（1秒）内的均方误差（MSE）小于1.6度，且在四小时的海洋环境数据上平均计算时间小于0.03秒，满足实时性要求。
    - 在仿真实验中，与传统的逆运动学（IK）控制器相比，所提方法在较高海况（3级海况）下，末端执行器的位置误差降低了42%，姿态误差降低了50%。

  - **实验验证：**
    - 在室内搭建了一个模拟USV基座扰动的实验平台，通过旋转平台模拟不同频率（0.2 Hz、0.5 Hz、1 Hz）的基座扰动。
    - 实验结果显示，在1 Hz的高频扰动下，所提方法相比IK控制器，末端执行器的位置误差降低了约30%，姿态误差降低了约58%。
    - 证明了所提方法在高频基座扰动下，能够有效地稳定机械臂末端，满足无人机着陆的要求。

  **关键词**

  - 修改的小波网络（MWN）, 模型预测控制（MPC）, 机械臂末端稳定控制, 受扰动基座上的机械臂, 无人机（UAV）着陆辅助, 轨迹生成与平滑, 海洋环境机器人, 关节速度约束, 系统带宽限制, 实时运动预测

---

- A manipulator-assisted multiple UAV landing system for USV subject to disturbance 
*R. Xu, C. Liu, Z. Cao, Y. Wang and H. Qian; Ocean Engineering 2024 Vol. 299 Pages 117306; DOI: 10.1016/j.oceaneng.2024.117306*

  **想解决的问题**

  本文旨在解决无人机（UAV）在受海浪扰动的无人艇（USV）上安全、准确着陆的问题。由于海浪引起的USV快速、不规则运动，传统的直接着陆方法面临挑战，要求较大的着陆平台才能保证着陆安全，限制了USV可携带的无人机数量。因此，作者希望设计一种系统，能够在有限的甲板空间内，实现多架无人机在受扰动的USV上安全着陆。

  **具体做法**

  *系统（设计）部分：*

  1. **Manipulator-Assisted Landing System（机械臂辅助着陆系统）**：设计并实现了一个基于机械臂的无人机着陆辅助系统。该系统包括一个七自由度的机械臂、捕捉器、无人机上的系留着陆系统和着陆平台。机械臂安装在USV上，负责跟踪并捕获无人机。

  2. **Catcher（捕捉器）设计**：机械臂末端配备了一个专门设计的捕捉器。捕捉器包含步进电机、磁铁阵列、PVC线等。磁铁阵列用于快速吸引无人机释放的对接块，PVC线通过旋转机构收紧，以锁定对接块，防止因USV扰动引起的连接中断。

  3. **Tethered Landing System（系留着陆系统）**：在无人机上安装了一个系留着陆系统，包括对接块、对接座和线轴。无人机在接近USV后，释放带有铁板的对接块。机械臂捕获对接块后，无人机通过线轴收紧绳索，实现安全着陆。对接结构设计成四面体形状，确保无人机在着陆过程中的姿态稳定。

  4. **Landing Platform（着陆平台）设计**：设计了一个可锁定和释放无人机的着陆平台。平台包含倾斜板、水平板和由舵机驱动的夹持器。机械臂将无人机精确放置在着陆平台上，夹持器旋转固定无人机，防止其滑落。

  5. **Visual Feedback System（视觉反馈系统）**：采用红外相机阵列构建视觉反馈系统，实时捕获无人机和机械臂的位置和姿态，反馈频率高达200 Hz，满足高速捕获任务的需求。

  6. **Control System（控制系统）**：基于ROS构建集中式控制系统，各设备通过无线网络连接，形成统一的控制和通信平台。机械臂的控制包括任务空间的MPC控制器和关节空间的PD控制器，并引入自适应估计器来补偿因基座运动引起的扰动。

  *算法部分：*

  1. **Uncertainty Estimator（不确定性估计器）设计**：针对基座扰动引起的机械臂控制精度下降问题，提出了一种自适应估计器。估计器通过在关节位置指令中加入补偿项，估计并补偿因基座运动引起的关节位置误差，而无需精确的动力学模型。

  2. **MPC Tracking Controller（MPC跟踪控制器）**：在任务空间中构建MPC控制器，实现对无人机位置的精确跟踪。控制器考虑了关节状态约束，通过预测模型优化控制输入，确保机械臂在有限的关节范围内安全运行。

  3. **Kalman Filter for Position Feedback（位置反馈的卡尔曼滤波器）**：为提高视觉系统的位置反馈精度，针对环境光干扰，采用卡尔曼滤波器对无人机位置进行滤波和预测，提高了系统的鲁棒性。

  4. **System Control Strategy（系统控制策略）**：制定了完整的着陆控制策略，包括三个阶段：
    - **跟踪和捕获阶段**：机械臂在任务空间中跟踪无人机，捕获其释放的对接块。
    - **系留着陆阶段**：机械臂稳定末端执行器，无人机通过收紧绳索实现安全着陆。
    - **精确放置阶段**：机械臂调整无人机姿态，将其精确放置在着陆平台上，完成着陆任务。

  **效果**

  - **仿真验证**：通过Matlab/Simscape和Robotics System Toolbox进行仿真，验证了提出的机械臂控制器的可行性。结果显示，引入自适应估计器的MPC控制器相比传统控制器，位置跟踪精度提高了约26.9%，姿态跟踪精度提高了约40.8%。

  - **室内实验**：在受控的室内环境下进行实验，验证了控制器的实际性能。结果表明，引入估计器后，机械臂的末端位置误差平均减少了65%，姿态误差平均减少了62%。

  - **野外实验证明**：在实际的水面环境下进行了系统功能测试。在USV受到约12度摇摆扰动的情况下，系统成功实现了无人机的捕获和着陆。实验验证了系统在真实海洋环境下的有效性和鲁棒性。

  **关键词**

  - 无人机安全着陆, 机械臂辅助着陆, 系留着陆系统, 不确定性估计器, 模型预测控制（MPC）, 自适应控制, 视觉反馈系统, 海洋环境机器人, 无人艇（USV）, 多无人机协同

---

- Confidence-Aware Object Capture for a Manipulator Subject to Floating-Base Disturbances 
*R. Xu, Z. Jiang, B. Liu, Y. Wang and H. Qian; IEEE Transactions on Robotics 2024 Vol. 40 Pages 4396-4413; DOI: 10.1109/tro.2024.3463476*

  **想解决的问题**

  本论文针对在波浪扰动下，无人艇（USV）上的机械臂捕获空中静止物体的挑战。由于波浪引起的基座准周期且快速的运动，导致物体运动预测精度较低。此外，机械臂受限的主动力矩使得通过实时追踪来准确捕获目标变得困难。传统的模型预测控制方法要求高精度的运动预测和足够的执行器力矩，但在随机的基座运动下，这些条件难以满足。作者希望开发一种在预测不精确的情况下，仍能规划可行的捕获轨迹的方法，以提高在浮动基座扰动下机械臂捕获物体的成功率。

  **具体做法**

  *算法部分：*

  1. **运动预测与置信度评估**

    - **波列网络（Wavelet Network, WN）预测：** 作者使用实时训练的波列网络来预测物体在基座坐标系下的运动轨迹。考虑到基座运动的非线性和不确定性，波列网络通过选取一组具有时频局部化特性的波列函数，实时回归运动模型。每个波列函数根据尺度和位移参数进行调整，能够有效捕捉波浪扰动下的瞬时变化。

    - **显著性项选择优化：** 为了提高计算效率，作者改进了波列网络中的显著性项选择过程。他们通过在正交分解过程中计算误差减少率（ERR），筛选出对预测贡献最大的波列函数，减少了不必要的计算。

    - **置信度分析：** 由于预测精度无法一直保持高水平，作者采用贝叶斯方法对预测结果进行置信度评估。将预测误差划分为不同的等级，根据观测到的误差等级，迭代更新预测的置信度。通过这种方法，得到一个多步预测的置信区间（置信管道），用于评估未来时刻的预测精度。

  2. **置信度感知的运动规划**

    - **捕获位置选择：** 考虑到预测精度随着预测步数的增加而降低，且机械臂需要一定的时间才能到达目标位置，作者提出了一个优化问题，以在捕获时刻 \( t_c \) 选择最佳的捕获位置。目标是最大化捕获位置的置信度，同时确保机械臂能够在给定时间内到达该位置。这个优化问题包括系统模型、任务空间误差和状态约束。

    - **运动轨迹规划：** 为了提高计算效率，作者将捕获位置和运动轨迹的规划问题分解为两个子问题：

      - **捕获位置优化：** 仅考虑捕获位置的可达性和置信度，建立一个非线性优化问题，决策变量为捕获时刻 \( t_c \) 和末端关节位置 \( q_N \)。通过约束关节位移不超过在 \( t_c \) 时间内的最大可移动距离，确保机械臂能够到达捕获位置。

      - **关节空间轨迹优化：** 在确定捕获位置后，规划机械臂从当前位置到捕获位置的关节空间轨迹。目标是最小化轨迹跟踪误差和控制输入，满足关节位置、速度、加速度等约束。这个问题被重新表述为一个二次规划（QP）问题，可以高效地求解。

    - **初始化方法：** 为了减少捕获所需的时间并提高成功率，作者提出了一种初始化位置选择方法。通过观测物体在基座坐标系下的运动，估计其运动区域，并在一个安全边界上选择一个初始化位置，使机械臂先移动到靠近物体的位置，缩短捕获时间。

  *系统（设计）部分：*

  - **实验平台：** 作者使用了一个冗余机械臂，末端配备了电磁铁，能够在接触物体时立即捕获。机械臂安装在伺服旋转平台上，以模拟浮动基座的扰动。

  - **视觉反馈系统：** 利用运动捕捉系统实时获取物体和基座的位姿信息，反馈频率为200 Hz，确保了高精度的定位。

  - **控制系统：** 控制系统基于ROS架构，实现了运动预测、置信度评估和运动规划等功能模块。机械臂在关节位置层面进行控制，控制频率为1 kHz，确保了控制的实时性。

  **效果**

  - **实验验证：** 作者通过超过150次实验，验证了所提方法的有效性。

    - **成功率：** 在基座执行真实无人艇运动（包括单轴和双轴的正弦扰动以及基于实际船只数据的不规则扰动）的情况下，所提方法实现了80%的捕获成功率。

    - **捕获精度提升：** 与传统方法相比，所提方法的平均捕获误差降低了25%以上。在不规则扰动下，捕获精度提高更为显著。

    - **算法效率：** 通过将捕获位置和轨迹规划问题分解，计算时间显著减少，所有计算在0.2秒内完成，满足了实时性的要求。

    - **鲁棒性：** 通过置信度评估和初始化方法，提升了在预测不准确情况下的捕获可靠性，证明了方法在复杂海洋环境下的适用性。

  **关键词**

  - 置信度分析, 波列网络（Wavelet Network）, 运动规划, 浮动基座机械臂, 物体捕获, 贝叶斯方法, 实时控制, 无人艇（USV）, 自动化系统, 不确定性处理


---

- Model-based Bending Control of Magnetically-actuated Robotic Endoscopes for Automatic Retroflexion in Confined Spaces
*Y. L. Yichong Sun, Jixiu Li, Wing Yin Ng, Yitian Xian, Yisen Huang, Philip Wai Yan Chiu, and Zheng Li; IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2023*

  ### 想解决的问题  
  1. 软尾导管-磁驱内镜在弯曲/翻折（retroflexion）时，软管弹性、外部磁场约束与狭窄腔道之间相互耦合，缺乏可靠的运动学模型与控制律。  
  2. 传统“U”型翻折需要较大回旋空间，遇到右侧结肠等狭窄段易失败，影响病灶检出率。  
  3. 现有控制多忽略软尾贡献或简单 PID，导致位置、姿态误差大，且难量化软管扫掠面积对黏膜安全性的影响。  

  ### 具体做法  

  #### 1. 软尾 + 磁头一体化运动学  
  * 软尾采用 Cosserat Rod 连续体模型：  
    * 中心线 P(s)、截面旋转 R(s)∈SO₃，弯-挠曲率 C(s)=RᵀR′。  
    * 假设无剪切/拉伸，材料线弹性，推导内部力 f_in、力矩 τ_in 的平衡微分方程 (3)(4)。  
  * 磁头刚体变换：P_t(m)=P(L)+mR_t i_z，内部力/矩沿 m 均衡到磁头中心；将磁场力-矩 w_e=[f_e;τ_e] 通过边界条件(8)(9)(10)耦合至软尾。  
  * 微分组由四元数形式 (11) 保证 R(s)∈SO₃，采用射线-Runge–Kutta shooting 数值求解。  

  #### 2. 模型叠加控制律  
  w_u = w_en + δw  
  * **误差-PID分量** δw：  
    * 位置误差 e_p=P_des−P_t，置于切向/法向投影矩阵 P_ep、P_ev；  
    * 姿态误差 e_o = R_t i_z × C_des。  
    * δw = [K_v e_v + K_p e_p ; K_o e_o]。  
  * **模型反馈分量** w_en = K_en [f_in(L); τ_in(L)] ，抵消软尾内力矩扰动。  
  * PID 与模型项并联：收敛时 δw→0，系统以 w_en 平衡。校准系数 K_en 补偿摩擦等非建模误差。  

  #### 3. “紧凑空间”自动翻折策略  
  1. 定义腔道中线 L_c；首 waypoint P_mid 位于 L_c，姿态与初始正交；  
  2. 当磁头达 P_mid, 切换到终点 P_end（同在 L_c）且姿态 180° 翻转；  
  3. 通过上式控制器逐段追踪 waypoint，实现磁头轨迹尽量贴近中线 → 最小扫掠面积。  

  ### 效果  

  | 指标 | 紧凑翻折 | 传统“U” | 降幅 |  
  |----|----|----|----|  
  | 最大横向偏移 L_Xmax | 60.8 mm | 86.7 mm | −30 % |  
  | 终点到中线距 D_Xlast | 16.3 mm | 78.7 mm | −79 % |  
  | 软管扫掠面积 S_approx | 2864 mm² | 5404 mm² | −47 % |  

  * 弯曲控制实验：目标 45°/90°/135°/180°，稳态位置误差 ≤3 mm，姿态误差 ≤4°。  
  * 翻折耗时约 50 s，可在∅55 mm 管道完成，大幅提高狭窄腔段可视范围。  

  ### 关键词  
  磁驱软体内镜；Cosserat Rod 模型；模型补偿 PID 控制；自动翻折；紧凑空间；扫掠面积














## 4. Machine Learning and related - 机器学习及相关 <a id="Machine_Learning"></a>

### 4.1 Supervised Learning - 监督学习 <a id="Supervised_Learning"></a>

### 4.2 Unsupervised Learning - 无监督学习 <a id="Unsupervised_Learning"></a>

### 4.3 Reinforcement Learning - 强化学习 <a id="Reinforcement_Learning"></a>

#### 4.3.1 Reinforcement Learning - 强化学习 <a id="Reinforcement_Learning"></a>

#### 4.3.2 Deep Reinforcement Learning - 深度强化学习 <a id="Deep_Reinforcement_Learning"></a>


### 4.4 Characterization/Modeling Techniques - 表征/建模技术 <a id="Characterization_Modeling_Techniques"></a>

#### 4.4.1 Deep Learning - 深度学习 <a id="Deep_Learning"></a>
























## 5. Computer Vision - 计算机视觉 <a id="Computer_Vision"></a>

- π^3: Scalable Permutation-Equivariant Visual Geometry Learning
*J. Z. Yifan Wang, Haoyi Zhu, Wenzheng Chang, Yang Zhou, Zizun Li, Junyi Chen, Jiangmiao Pang, Chunhua Shen, Tong He; 2025*

  ### 想解决的问题  
  1. 现有前馈 3D 重建网络（VGGT、VGGSfM、DUSt3R 等）必须人为指定“参考视图”，并把所有几何预测到该坐标系。  
    • 参考帧若选得不好会导致性能大幅波动，训练-推理都对输入顺序敏感，难以扩展到海量视图或动态场景。  
  2. 相机-点云的尺度/仿射不变性被硬编码到参考系后，网络学习空间被限制，收敛慢、鲁棒性差。  
  3. 需要一个完全“对输入排序等变(permutation-equivariant)”的架构，可在无参考帧、任意视图数目下高效输出：  
    - 仿射不变相机位姿 - 尺度不变局部点云 - 置信度等，覆盖静/动态、室内/外、多域数据。

  ### 具体做法  

  #### 1. 架构设计：œÄ³ (PI3) Transformer  
  1. 去除一切顺序偏置  
    * 不使用帧 index positional embedding、不拼接 camera token。  
    * 输入 N 张图，各自 patchify→DINOv2 token。  
  2. 交替注意力模块（与 VGGT 类似）  
    * View-wise Self-Attn：帧内局部信息。  
    * Global Self-Attn：跨帧几何一致性。  
    * 完全自注意，无 cross-attn，层数 36（L 变体）。  
  3. Decoder（共享结构不同权）  
    * 5 层轻量 Transformer，仅对单帧 token 自注意。  
    * 三个 head：  
      - Camera Head：Reloc3r 结构输出 4×4 仿射位姿 Ti（旋转 9D→SVD 正交化）；  
      - PointMap Head：MLP+PixelShuffle 输出 X_i ∈R^{H×W×3}（相机坐标系，未知尺度）；  
      - Confidence Head：同结构输出 C_i ∈R^{H×W}.  

  #### 2. 目标表示  
  * **尺度不变局部点云**：每帧独立坐标，训练时全序列共享单一尺度 s★ 与 GT 对齐（ROE solver）。  
  * **仿射不变相机**：仅监督两帧间相对位姿 (R, t)，用 s★ 修正平移。  

  #### 3. 损失函数  
  ```
  L = Lpoints + λn Lnormal + λc Lconf + λcam Lcam
  ```
  * Lpoints：∑ (1/z)·|s★·x̂ - x|_1  
  * Lnormal：像素邻域叉乘→角度差  
  * Lconf：BCE，阈值 ζ 上下界。  
  * Lcam = mean_{i≠j}( geodesic(R̂,R) + α·Huber(s★ t̂ - t) ).  

  #### 4. 训练与数据  
  * 15 个多模态数据集混合（GTA-SfM, CO3D, ScanNet++, HyperSim, MegaDepth, MatrixCity…+内部动态集）。  
  * 两阶段：224²（100 epoch）→随机分辨率 ≤255k pix（100 epoch）；DINOv2 编码器冻结。  
  * Adam lr 1e-4 Cosine；动态 batch 64→48；A100×64，bfloat16，约 2×1000 iter/epoch。  

  #### 5. 推理流程  
  * 完全前馈：N 图 → {(T_i, X_i, C_i)}，57 FPS(Hidden-L, H×W≈255k)。  
  * 可选 Sim(3) BA 微调，相比 VGGT 仅 1.6 s（因已给初值，无三角化）。  

  ### 效果  

  1. 相机姿态  
    * RealEstate10K AUC@30 = 85.9（零样本，↑8% 优于 VGGT）。  
    * Sintel ATE 0.074（VGGT 0.167）；TUM-dyn/ScanNet 持平 VGGT。  
  2. 点云 (Scale-free)  
    * DTU Overall ↓1.198 mm（Mean），优于 VGGT(1.338) Fast3R/CUT3R。  
    * ETH3D Overall ↓0.210 mm（Median 0.128）。  
    * 7-Scenes sparse 0.048/0.072 (Acc/Comp) ，dense 0.015/0.022，均 SOTA。  
  3. 视频 / 单目深度  
    * 视频深度 AbsRel 0.233 (Sintel)、0.038 (KITTI)；帧速 57 FPS。  
    * 单目深度在四数据集逼近专用模型 MoGe。  
  4. 鲁棒性  
    * 将序列首帧换 N 次，DTU/ETH3D 度量 std ≈ 0（VGGT 0.03-0.06）。  
  5. 可扩展性  
    * 模型尺寸 S/B/L 提升收敛速度 & 性能（Large 比 baseline 提升45%，收敛 epoch↓）。  

  ### 关键词  
  Permutation-Equivariant Reference-free 3D Affine-Invariant Camera Scale-Invariant Point Map Alternating Self-Attention Feed-forward Visual Geometry

---

- VGGT: Visual Geometry Grounded Transformer
*M. C. Jianyuan Wang, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, David Novotny; CVPR 2025*

  ### 想解决的问题  
  * 3D 视觉传统依赖 SfM/MVS 等迭代几何优化，流程复杂、耗时长（秒~分钟），且往往每一子任务（相机、深度、点云、跟踪）需要专门模型/后处理。  
  * 现有端到端网络多专注单一任务（DepthAnything、LRM 等）或只能处理两视图（DUSt3R/MASt3R），难以一次性从任意数量图像直接输出完整 3D 属性。  
  * 亟需一种统一、前馈、可扩展的大模型作为 3D 视觉通用“骨干”，像 NLP GPT、Vision CLIP/DINO 那样，做到“一次前向→相机+深度+点云+轨迹”，并对下游任务具备强迁移能力。  

  ### 具体做法  

  #### 1. 架构：Visual Geometry Grounded Transformer (VGGT)  
  1. Patchify：DINOv2 提取 14×14 Patch tokens；每帧再附加 1 个 Camera token + 4 个 Register tokens。  
  2. Alternating Attention (AA)：  
    * Frame-wise Self-Attn：仅在单帧内建模局部语义。  
    * Global Self-Attn：在所有帧 token 间交叉信息。  
    * 交替堆叠 24 层（总参数 12 亿），无 Cross-Attn、无 3D 归纳偏置，仅依靠大数据学习几何。  
  3. Token 译码：  
    * 相机 Head：对 Camera tokens 过 4 层 Self-Attn + FC，输出 q(四元数), t, f 共 9 维。第一帧固定为世界坐标。  
    * DPT Head：将图像 tokens Upsample 成特征 F_i →  
      - Depth map D_i（+ aleatoric σ_D）  
      - Point map P_i（世界坐标，预测冗余但多任务互促）  
      - Tracking feature T_i（C=128）  

  #### 2. 训练机制  
  * 数据：汇聚 15+ 数据集（Co3Dv2、ScanNet、MegaDepth、Kubric、OBJaverse 类合成等），室内/室外/合成混合。  
  * 批次：随机抽 2–24 帧，保证总帧数 48；最大边 518px，Aggressive 颜色增强。  
  * 损失：  
    ```
    L = L_camera + L_depth + L_point + 0.05 L_track
    ```  
    * Camera 9 维 Huber；Depth/Point 使用 Aleatoric σ&梯度一致性；Track 用 CoTracker2 双向误差+BCE可见性。  
  * 归一化：GT 点云以第一帧坐标&平均尺度归一；网络预测不强制归一，使模型学习绝对尺度。  
  * 优化：AdamW lr 2e-4，160k iter，cosLR；64×A100，bfloat16+Gradient Checkpoint，训练 9 天。  

  #### 3. 推理流程  
  前向一次即可：  
  输入 N(1–200) 张图→0.2–9 s；输出 N 组 {相机 g_i, 深度 D_i, 点云 P_i, Track feat T_i}。  
  若需极致精度，可用 BA 对 g_i / 3D 点做一次性微调（~1.6 s），因 VGGT 已给出高质量初值，不需三角化。  

  ### 效果  

  1. 相机估计  
    * RealEstate10K AUC@30↑85.3；CO3Dv2 ↑88.2；均超 VGGSfMv2/MASt3R +10~20%。  
    * BA 微调后 IMC AUC@10 提升至 84.9，夺得 CVPR’24 IMC 榜首。  
  2. 深度 / 点云  
    * DTU Overall 0.382（无 GT camera），逼近 GeoMVSNet(0.295, 用 GT camera)。  
    * ETH3D Chamfer 0.677，明显优于 DUSt3R/MASt3R，并快 30×。  
  3. 两视图匹配（ScanNet1500）AUC@10 55.2，超 Roma/LoFTR。  
  4. 动态点跟踪（TAP-Vid RGB-S）微调 CoTracker 后 AJ 提升至 72.1(+4.7)。  
  5. Novel View Synthesis（GSO，小数据+无输入相机）PSNR 30.4，接近 LVSM(31.7, 需相机)。  
  6. 速度&扩展：H100 上 100 帧 3.1 s / 21 GB；单张亦支持单视重建。  

  ### 关键词  
  大型 3D 基座模型；前馈多视几何；Alternating Attention；相机姿态估计；多视深度；稠密点云；3D 点追踪；下游迁移

---

- 4-DOF Visual Servoing of a Robotic Flexible Endoscope With a Predefined-Time Convergent and Noise-Immune Adaptive Neural Network
*Y. Huang, W. Li, X. Zhang, J. Li, Y. Li, Y. Sun, et al.; IEEE/ASME Transactions on Mechatronics 2024 Vol. 29 Issue 1 Pages 576-587; DOI: 10.1109/tmech.2023.3286850; https://dx.doi.org/10.1109/tmech.2023.3286850*

  ### 想解决的问题  
  1. 传统内窥镜助手手持操作存在疲劳与误差；现有机器人内窥镜多数只做 2-DOF 视野居中，无法同时调节距离与旋转，导致“视向错位(misorientation)”——器械在画面中角度、尺寸不合适。  
  2. 内窥镜电缆驱动柔性关节模型不精确（死区、迟滞），加之目标检测易受遮挡、模糊等视觉噪声干扰，常规控制器易发散或产生抖振。  
  3. 在微创手术场景下还需满足 RCM（穿孔铰点）约束和关节速度/角度物理限幅，传统优化器速度不足，且现有神经网络优化解算缺乏预设时间收敛与抗周期噪声能力。  


  ### 具体做法  

  #### 1. 系统硬件与任务  
  * 10-DOF 机构：KUKA LBR Med 7 机械臂 + 3 电机驱动柔性内窥镜（2 × 弯曲，1 × 轴向旋转）。  
  * 端镜头 1920×1080@60 Hz，检测四点矩形包围器械尖端。  

  #### 2. 4-DOF 图像特征设计  
  s = [u_g, v_g, a_n, α]ᵀ  
  * u_g,v_g：质心坐标 → 控制平移（2-DOF）  
  * a_n：二阶中心矩 μ20+μ02 归一化 → 控制深度（缩放）  
  * α：器械长轴方向 → 控制镜头自旋  

  Ji 映射 s˙ = Ji · [c v_c ; ω_cz]，与机械臂 Jacobian Jr 拼成 J_sys。  

  #### 3. 误差学习型滑模控制（EL-SMC）  
  Sliding surface: s_f = Λ[e_sᵀ, e_rᵀ]ᵀ (e_r 为 RCM 误差 xy)  
  Reaching law:  
  ṡ_f = −K_s s_f − Γ f_t(s_f)  
  * f_t(·)：基于三段模糊隶属 μ₁,μ₂,μ₃ 的平滑切换函数，抑制抖振。  
  * Γ = diag{|Φ(Z)|}，Φ 由 GRBFNN 在线输出；权值自适应律 ˙Ŵ = −ψ(g s_fᵀ + λ Ŵ)。 → 大误差高增益、稳态小增益。  

  #### 4. 约束运动生成为 QP  
  min ½‖q̇ + β(q−q₀)‖²_W  
  s.t.  ΛJ q̇ = K_s s_f+Γ f_t  
        q̇⁻ ≤ q̇ ≤ q̇⁺ ,  q⁻ ≤ q ≤ q⁺  
        RCM 侧向速度零化 A_rcm q̇ = 0  
  → 转为标准 QP: min xᵀWx/2 + ĉᵀx, Ax=b, Cx≤d  

  #### 5. 预定义时间收敛-抗噪自适应零点神经网络 (AZNN)  
  连续动态：  
  F ẏ + v̇ = −ϕ_adapt(en) Ψ(en) + N  
  * en = F y+v 为 KKT 误差，ϕ_adapt = κ₁ + κ₂(1−‖en‖^ι/χ)  
  * 新激活函数 ϕ_n(x)=p₁ e^{|x|^g}|x|^{1−g}sgn(x)/g + p₂x³ + p₃sgn(x)  
    - g∈(0,1)，保奇增；保证在 t_p ≤1/(p₁κ₁) 内收敛且对有界周期噪声 Σ 抗扰（Σ≤p₃κ₁）。  

  #### 6. 控制循环  
  60 Hz 视觉→125 Hz 控制：  
  1. 检测器给 s, 计算 e_s。  
  2. EL-SMC 输出等式约束右端。  
  3. 将 QP 离散化 (τ=8 ms)，AZNN 迭代 k_max=5000 或 ‖e_n‖<1e-3。  
  4. 得 q̇ 指令→KUKA 1 kHz 速度接口。  


  ### 效果  

  1. **控制性能**  
    * 对比 PID、经典 SMC、平滑 SMC：本方法收敛时间最快（<2 s），稳态无抖振，RCM 误差 <1 mm。  
  2. **4-DOF vs 2-DOF**  
    * 追踪手术器械沿 8 字路径 35 s，4-DOF 最终画面旋转误差 <2°；2-DOF 累计旋转 28°，出现视向错位。  
  3. **鲁棒性**  
    * 轴杆碰撞、柔性段扰动、目标遮挡三种干扰均快速恢复，系统稳定。  
  4. **优化求解比较**  
    * 与 DNN、RNN、GNN、IPM 等：AZNN 计算平均 0.18 ms/loop，误差 5e-4；其他方法>1 ms 或无法收敛。  
  5. **模拟胸腔猪肺切除**  
    * 5 块组织切除成功，用时 25 s；FOV 始终稳定，RCM 偏差 <2.5 cm。  



  ### 关键词  
  柔性机器人内窥镜；4-DOF 视觉伺服；视向错位；滑模控制；自适应 RBF 神经网络；预设时间收敛；抗周期噪声；二次规划神经解算；RCM 约束

---

- A Simultaneous Polyp and Lumen Detection Framework Toward Autonomous Robotic Colonoscopy
*Y. L. Wing Yin Ng , Tianle Pan , Yichong Sun, Qi Dou , Pheng Ann Heng , Philip Wai Yan Chiu , and Zheng Li; IEEE TRANSACTIONS ON MEDICAL ROBOTICS AND BIONICS 2024 Vol. 6*

  ### 论文总体脉络与关联性分析  
  该工作围绕“提升机器人结肠镜自主化”这一核心目标，提出了一个端到端的深度学习框架，用于同时检测腔腔中心 (lumen) 与息肉 (polyp)，并将识别结果闭环驱动磁控柔性结肠镜（EAST）完成自主导航与诊断。论文通过“数据—算法—系统—验证”四层递进设计，建立了从基础数据集构建到体外/离体实验验证的完整链路，可为后续任何面向“视觉感知 + 运动控制”闭环医疗机器人的研究提供参考。  

  #### 1. 想解决的问题  
  * 传统柔性结肠镜操作困难、病人痛苦，现有机器人结肠镜自治级别普遍偏低。  
  * 在自治导航和实时诊断两个关键环节，缺乏能同时稳健检测“息肉 + 腔腔中心”的统一视觉感知模块。  
  * 公开标注的“腔腔中心”数据集几乎为空白，阻碍算法开发；现有图像处理方法需大量人工调参，鲁棒性差。  

  #### 2. 具体做法  
  **（A）算法层**  
  1. 数据集构建  
    * 扩展公开 LDPolypVideo 得到 **Extended-LDVideo**（40,186 张，双标注：息肉 + 腔腔中心）。  
    * 自制硅胶结肠 **Phantom 数据集**（2,000 张，高分辨率 1280×720），用于实验室场景验证。  
  2. 检测模型基线  
    * 评测 5 类主流检测器：Faster-RCNN, RetinaNet, SSD(MobileNet-v2/v3), YOLO-v3, **YOLO-v8**。  
    * 采用相同训练/评估协议（MMDetection + Pytorch，IoU 0.5-0.95）。  
  3. 数据混合策略  
    * 提出 **Supported-to-Intended (SI) Ratio** 概念，系统研究“临床-仿真”数据配比对 mAP 的影响：  
      - 发现仅 25% 真实数据 + 100% 仿真数据即可把模型性能由“不可用”跃迁到“可用”区间。  
  4. 最终算法  
    * 选用 **YOLO-v8 + 自行调优损失权重** 作为统一检测器：  
      - Extended-LDVideo：mAP 0.841，Recall 0.866，89 FPS。  
      - Phantom：mAP 0.896，Recall 0.887。  

  **（B）系统/设计层**  
  1. **EAST 软牵引磁控结肠镜**  
    * 体外 8 线圈电磁阵列产生 3D 磁场；利用磁偶极模型实时解算牵引力/力矩。  
  2. 视觉-运动闭环  
    * YOLO-v8 输出：  
      - 若检测到息肉 → 置目标为息肉中心，进入诊断/观察模式。  
      - 否则 → 目标为最近腔腔 haustral fold 中心，进入导航模式。  
    * 位置误差 → MPC 控制器求解所需磁力 → 电流指令发送至电磁阵列。  

  #### 3. 效果  
  * **检测精度**：YOLO-v8 在双数据集均获最高 mAP；误检/漏检率显著低于 darkest-region、contour、混合传统算法。  
  * **鲁棒性**：动态旋转实验（-20°~20°）下腔腔中心 MAE 3.95%，优于 darkest-region (14.38%)，与混合方法(3.10%)接近，但推理速度快 3×。  
  * **自主导航**  
    - 硅胶 Phantom：平均像素误差 (u/v) 5.38% / 3.98%，速度 16.3 mm/s，可自动切换“导航→息肉跟踪→导航”。  
    - 离体猪结肠：平均误差相似，速度 8.17 mm/s。  
  * **开放资源**：代码与两数据集已在 GitHub 开源，为社区提供首个>4 万张的“息肉+腔”双标注数据。  

  #### 4. 关键词  
  深度学习；腔腔中心检测；息肉检测；机器人结肠镜；YOLO-v8；Supported-to-Intended Ratio；磁控柔性内镜；自主导航；医疗机器人；数据集构建

---

- Colon Lumen Center Detection Enables Autonomous Navigation of an Electromagnetically Actuated Soft-Tethered Colonoscope
*W. Y. N. Yehui Li, Yichong Sun, Yisen Huang, Jixiu Li, Philip Wai Yan Chiu, Zheng Li; IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT 2024 Vol. 73*

  ### 论文整体关联性与脉络分析  
  作者围绕“让磁控软牵引结肠镜（EAST）在真实肠腔中自行导航”这一目标，构建了一条完整闭环：  
  1. **感知层**——提出混合式腔腔中心检测：用改进轻量级深度网络提取结肠褶皱轮廓，再融合暗区中心；解决纯暗区或纯边缘法鲁棒性不足问题。  
  2. **决策层**——根据像素误差设计模糊‐滑模控制器与自适应速度律，实现带扰动的稳健对准。  
  3. **执行层**——利用 9 线圈电磁阵列 + 二自由度软体套管，把期望姿态、牵引力转化为最优线圈电流。  
  4. **验证层**——在商业仿真肠道、离体猪肠、大样本用户实验中，量化“更快、更稳、更轻负”的优势。  
  这一链路清晰展示了“视觉感知—运动控制—系统实现—临床可行性”的逻辑闭环，可为其它腔镜/软体机器人提供通用模板。

  #### 1. 想解决的问题  
  * 现有机器人结肠镜导航仍依赖人工操控，过程耗时、负担大。  
  * 单纯 “最暗区域” 或 “边缘提取” 的腔腔中心估计在照明变化、锐角弯曲等场景下失效，导致早调头或碰壁。  
  * 缺乏能在真实与模拟环境均稳定工作的自主导航控制策略。  

  #### 2. 具体做法  
  **算法部分**  
  1. 数据集与标注  
    * 自建 600 张 RGB 结肠褶皱轮廓数据集（300 张公开 LDPolypVideo + 300 张医院临床视频），逐像素标注主要褶皱边缘。  
  2. 轮廓分割网络  
    * 基于轻量级 **LDC** 编码器，提出 **UpDecoder**：  
      - 三分支深度可分离卷积 + 级联融合，逐层上采样聚合多尺度边缘特征。  
      - 输出 640×480 图像实时 46 FPS。  
  3. 最暗区域检测  
    * 灰度取反 → 阈值分割（动态阈值 = 0.8×最大灰度）→ 最大连通域 → 计算质心。  
  4. 最终腔腔中心筛选  
    * 先按面积 >500 px 过滤噪声边缘；  
    * 保留包络“暗区质心”的最大边框；  
    * 若轮廓缺失，则回退使用暗区质心。  

  **系统（设计）部分**  
  1. **EAST 软牵引结肠镜**  
    * 9 mm 软管内嵌 13 mm×20 mm 轴向磁钢 + IMU + 3.9 mm 摄像头。  
    * 工作空间由 9 线圈电磁阵列生成 3D 磁矩与牵引力。  
  2. 建模  
    * 2-OR PRB 软体杆模型：关节角 θ₂, θ₃ 描述头端偏转；  
    * 小孔相机模型建立像素‐姿态雅可比 J；  
    * 磁偶极模型 + 二次规划求解最小能耗线圈电流。  
  3. **自主导航控制**  
    * 滑模面 s=e=[u−u₀, v−v₀]ᵀ；  
    * 控律：˙θ = J† ẏ(κs + ℘ sat(s))，其中 ℘ 由 Takagi-Sugeno 模糊逻辑自调避免抖振；  
    * 速度律：χ = (1 − ln(p‖e‖²+1)/ln(p‖e‖²_max+1)) χ_max，实现“误差大减速、误差小提速”。  

  #### 3. 效果  
  * **轮廓分割性能**：F-score(ODS/OIS) 0.807/0.823，AP 0.826；优于 Canny、HED 等基线，依旧 46 FPS。  
  * **腔腔中心检测**  
    - 静态误差：u,v 坐标平均 2.67% / 2.71%，优于暗区法和传统边缘法；  
    - 动态旋转 (±20°)：归一化 MAE 4.72%。  
  * **姿态控制**：带气流干扰 1.75 s 收敛，无扰 1.5 s；抖动 <5 px，明显优于纯比例控制。  
  * **自主导航**  
    - 仿真肠道 0.6 m：平均 41.6 s 完成；  
    - 离体猪肠 0.6 m：31.2 s 完成；持续对准误差 <10 px。  
  * **用户研究（2 新手 + 2 专家）**  
    - 本方法平均 44.9 s，显著快于手动（CFC/EAST）；  
    - NASA-TLX 各维度负荷最低，尤其“精神/体力需求”与“挫败感”下降 >50%。  

  #### 4. 关键词  
  结肠腔中心检测；混合深度学习；UpDecoder；模糊滑模控制；自适应速度；磁控软牵引结肠镜（EAST）；视觉伺服；自主导航；用户负荷评估


---


- Noise-Consistent Siamese-Diffusion for Medical Image Synthesis and Segmentation
*Z. G. Kunpeng Qiu, Zhiying Zhou, Mingjie Sun, Yongxin Guo; CVPR 2025 2025; DOI: 10.48550/arxiv.2505.06068; https://dx.doi.org/10.48550/arxiv.2505.06068*

  ### 想解决的问题  
  1. 医学图像分割依赖大量标注数据，但真实数据昂贵且隐私受限，常用的扩充方案——掩膜先验条件的扩散模型，因训练数据同样稀缺，生成图像形态逼真度（texture / morphology）低，削弱增强效果。  
  2. 直接加入图像+掩膜双先验虽能提高逼真度，但采样阶段必须提供配对图像，导致多样性差、可扩展性受限，无法真正解决数据匮乏。  



  ### 具体做法  

  #### 1. Siamese-Diffusion 总体框架  
  同一扩散 U-Net 在训练时并行执行两条分支：  
  • Mask-Diffusion：仅以掩膜 y₀ 生成噪声预测 ε_m_θ。  
  • Image-Diffusion：以图像 x₀ + 掩膜 y₀ 混合先验 (c_mix) 生成 ε_mix_{θ′}。  
  两分支共享参数（θ′ 为 θ 的深拷贝），引入三种损失共同优化：  

  1) 掩膜去噪损失 L_m = E‖ε_m_θ − ε‖²  
  2) 图像去噪损失 L_i = E‖ε_mix_{θ′} − ε‖²  
  3) 噪声一致性损失 L_c = w_c E‖ε_m_θ − sg[ε_mix_{θ′}]‖²  
    • stop-gradient 使 ε_mix 作为“锚点”在参数空间引导 θ 朝高形态逼真度极小值。  

  在线增强：利用 ε_mix 单步逆扩散 (DDIM like) 得到 z′₀，与 y₀ 重新组合训练 Mask-Diffusion（损失 L_m′，权重 w_a 受迭代/时间步阈值控制）。  

  最终推理阶段仅用 Mask-Diffusion，可接受任意掩膜，兼顾多样性与高保真。  

  #### 2. Dense Hint Input (DHI) + ControlNet 特征提取  
  原稀疏 HintInput 适合低密度先验。本工作替换为 DHI：5 级残差块（16→256通道）+ Patch Merging，使高密度医学图像与掩膜特征均能有效注入。DHI 与 ControlNet 串联构成外部 HyperNetwork，参数与 U-Net 共同学习。  

  #### 3. 先验混合策略  
  c_mix = w_i·c_img + w_m·sg[c_mask]，其中 w_i = k/N_iter 逐步增大，前期稳定收敛，后期放大图像先验作用。  

  #### 4. 损失权重与超参  
  • w_c（IFG指导强度）=1.0 最优，过大/过小都会降低分割提升；  
  • Online-Aug 阈值 K_τ=N_iter/3，T_τ=200；  
  • 训练：StableDiffusion-v1.5 冻结 VQ-VAE/Encoder，AdamW lr 1e-5，8×4090 batch=48，50 DDIM steps 采样。  


  ### 效果  

  1. 图像质量（Polyps 数据集）  
    • FID 62.7、KID 0.039、LPIPS 0.586，较 SOTA ArSDM 提升 35 FID；t-SNE 与真实分布几乎重叠。  
    • 临床专家 MOS 0.587（真实图像 0.9），优于 ControlNet 0.487。  

  2. 分割提升  
    • Polyps 五测试集平均：SANet mDice↑3.6%、mIoU↑4.4%；Polyp-PVT mDice↑1.1%；CTNet mDice↑0.9%。  
    • ISIC2018：UNet mDice↑1.52%，SegFormer mIoU↑1.02%；随机形变掩膜多样性进一步提升 0.8~1% 指标。  
    • 无真实像素场景（Stain/Faeces）Synth 1000 张即可超越真实训练集，显示可扩展性。  

  3. 消融验证  
    • DHI、Online-Aug、L_c 各自贡献：形态对齐、色彩/纹理、纹理细节。三者协同效果最佳。  
    • 不同 w_c 实验：0→质量低，多样性好；0.5~1.5 区间性能最优，>2 开始退化。  
    • 数据量实验：3× 合成数据性能峰值，再增则出现“长尾”下降。  

  ### 关键词  
  医学图像生成；扩散模型；掩膜先验；Siamese-Diffusion；噪声一致性损失；高形态逼真度；数据扩充；医学分割增强

## [3DGS]
- EndoVLA: Dual-Phase Vision-Language-ActionModel for Autonomous Tracking in Endoscopy
*L. B. Chi Kit Ng, Guankun Wang, Yupeng Wang, Huxin Gao, Kun Yuan, Chenhan Jin, Tieyong Zeng, Hongliang Ren; CoRL 2025; DOI: 10.48550/arxiv.2505.15206; https://dx.doi.org/10.48550/arxiv.2505.15206*

  ### 想解决的问题  
  1. 传统内镜自动跟踪依赖“检测-规划-控制”流水线，需大量人工调参，对光斑反射、组织变形等复杂场景极为脆弱，且难融入医生的高层语义意图。  
  2. 现有 RL / IL 方法数据需求大、奖励设计繁琐，难以跨任务、跨场景泛化到实际胃肠道动态环境。  
  3. 通用多模态大模型虽可理解文本与图像，但直接用于内镜机器人存在语义鸿沟，安全/确定性不足。  


  ### 具体做法  

  #### 1. 整体框架：EndoVLA  
  输入：单帧内镜 RGB + 医生语言指令  
  输出：目标框 [x,y,w,h] 与离散弯曲动作 **At** ∈{UR, UL, LL, LR, Stop} → 两电机增量 Δθ = [±δθ1, ±δθ2]。  

  核心三任务  
  1. Polyp Tracking (PP) 2. Abnormal-Region 跟踪 (AR) 3. Circular-marker Cutting (CC)  

  #### 2. 双阶段微调 (Dual-phase Fine-Tuning, DFT)  
  1. SFT (Supervised Fine-tune)  
    * 基座：Qwen2-VL-7B 冻结视觉编码器（ViT）  
    * LoRA 适配器（4× r=16）插入 Transformer，MLP projector 将 Ev(Ot) 对齐语言嵌入。  
    * 损失 = Lbbox(ℓ1 + IoU) + Laction(CE)。  
  2. RFT (Reinforcement Fine-tune)  
    * 引入 **Group Relative PPO (GRPO)**，以目标-视图分组稳定更新。  
    * 可验证奖励：  
      - IoU 奖励 R₁ = IoU(Bpred,Bgt)  
      - Motion-Angle 奖励 R₂ = 1 / 0  
      - 格式奖惩 R₃ = 1 / 0  
    * 总奖励 R = w₁R₁+w₂R₂+w₃R₃。  
    * 采样 g=4 组轨迹；裁剪 ε=0.2；KL 惩罚 α=0.01。  

  #### 3. EndoVLA-Motion 数据集  
  * 6 k 图-语-动作对（400×400, 30 fps），来源：两款胃模型 + Olympus 2-DOF 软内镜机器人。  
  * 自动 YOLOv5 标注 + 人工清洗；三任务动作分布如表：  
    ```
    PP  1053  AR 1130  CC 2833   (训练80%,评估20%)
    ```  
  * 三类 prompt  
    - Ia：直接输出动作  
    - Ib：输出框+动作  
    - Id：输出方位描述+动作  

  #### 4. 机器人执行  
  * Olympus 内镜手柄两旋钮 → 两电机闭环控制，假设弯曲角 α∝θ。  
  * 控制循环：30 Hz 摄像—> EndoVLA 2 Hz 推理—> 低通补偿映射至 Δθ —> 电机。  

  ### 效果  

  1. **消融**  
    * SFT vs RFT vs DFT (IoU / Precision)：DFT 提升 PP +38.6 %、AR +64.1 %、CC +340.9 %。  
  2. **机器人实测 (30 次/任务)**  
    * PP、AR　成功将目标移入关注圆 FR：63 % / 57 %（Ia），均比单阶段高 >110 %。  
    * CC　完整沿环割线跟随成功率 10 %，其余方法 0 %。  
  3. **零样本泛化**  
    * CORL 字符-图标序列、果蔬序列、户外洞口跟踪：EndoVLA 成功率 50~100 %，RFT 0 %。  
  4. **基座模型比较**  
    * 仅 Qwen2-VL 推理：动作精度 <51 %，加入 GT 框亦不足。EndoVLA-DFT 达 95 %+。  

  ### 关键词  
  内镜机器人；Vision-Language-Action；双阶段微调；可验证奖励强化学习；多模态大模型；连续体机械臂自主跟踪


















## 6. Large Language Model - 大语言模型 <a id="Large_Language_Model"></a>















## 7. Vision Language Model/Vision Language Action Model - 视觉-语言模型/视觉-语言-动作模型 <a id="Vision_Language_Model/Vision_Language_Action"></a>

- π0.5: a Vision-Language-Action Model with Open-World Generalization
*N. B. Kevin Black, James Darpinian, Karan Dhabalia, Danny Driess, Adnan Esmail, Michael Equi,, N. F. Chelsea Finn, Manuel Y. Galliker, Dibya Ghosh, Lachy Groom, Karol Hausman, Brian Ichter,, T. J. Szymon Jakubczak, Liyiming Ke, Devin LeBlanc, Sergey Levine, Adrian Li-Bell, Mohith Mothukuri,, K. P. Suraj Nair, Allen Z. Ren, Lucy Xiaoyang Shi, Laura Smith, Jost Tobias Springenberg, Kyle Stachowicz and Q. V. James Tanner, Homer Walke, Anna Walling, Haohuan Wang, Lili Yu, Ury Zhilinsky; 2025; DOI: 10.48550/arxiv.2504.16054; https://dx.doi.org/10.48550/arxiv.2504.16054*

  ### 想解决的问题  
  1. 当前 Vision-Language-Action (VLA) 端到端策略往往只能在采集数据的实验室或少量家庭场景中工作，缺乏“开世界”泛化能力。  
  2. 单一来源的数据（仅一台机器人、仅低级动作演示等）难以覆盖真实家庭中不同房型、物体类别和长时序任务（10-15 min）的复杂性。  
  3. 缺少统一框架将多模态网页知识、跨机器人数据、高层语义分解与连续低层控制高效融合。  

  #### 1. 数据与训练配方  
  A. 数据源异构融合  
    * MM：400 h 移动机械臂在约 100 套真实住宅收集的清洁、收纳数据  
    * ME：多地固定机械臂（单/双臂）在真实厨房/卧室采集，弥补机体差异  
    * CE：实验室跨机体数据集（含公开 OXE）覆盖折衣、扫桌等丰富动作  
    * HL：为上述机器人数据人工标注子任务文本（如“pick up plate”）与目标物框  
    * WD：网页图像-文本任务（caption、VQA、检测）≈97 % 预训练样本  
    * VI：专家用自然语言逐步指挥机器人完成任务的“语言演示”  

  B. 双阶段协同训练  
    1) 预训练阶段  
        • 所有动作先经 FAST 压缩成离散 token，与图像 patch、文本指令统一序列 → 标准自回归 Transformer 训练（280 k 步）。  
    2) 后训练阶段  
        • 引入 Flow-Matching 动作专家（300 M 参数）输出连续 50-step action chunk；  
        • 统一损失  
          L = CE(token) + α‖ω – a – f_FM(·)‖², α=10  
        • 仅用与移动操控最相关的 MM+ME+HL+VI+WD 再训练 80 k 步，实现高频推理 (50 Hz)。  

  #### 2. 模型架构  
  1. 主干 VLM：SigLIP-400 M 图像编码 + Gemma-2 B 语言解码。  
  2. 多模态序列设计  
    * 输入：图像 patch ×4 视角 + 机器人 proprio 量化 token + 任务 prompt  
    * 输出：①高层子任务文本 token ˆℓ；②动作专家生成连续 aₜ:ₜ₊₄₉  
  3. 注意力遮罩  
    * 图像/文本可双向；FAST 动作仅看 prefix；动作专家 tokens 仅看 prefix & 内部互看，避免信息泄漏。  

  #### 3. 层级推理流程  
  1. 高层推理（1 Hz）：π(ˆℓ | o,ℓ)  → 预测下一语义子任务 & 相关物体框  
  2. 低层推理（50 Hz）：π(aₜ:ₜ₊₄₉ | o,ˆℓ) → Flow-Matching 迭代 10 步求动作 chunk  
  3. 直接用 PD 控制跟踪：两臂 6 DoF+夹爪+升降+全向底盘，共 18/19 DoF，无额外规划。  


  ### 效果  
  1. **跨域泛化**：在 3 套未见过的真实住宅中，指令“clean kitchen/bedroom”一次执行 10–15 min，多阶段成功率 ≈80 %，可自动关柜门、擦台面、放碗盘、整理床铺等。  
  2. **规模实验**：训练住宅数从 3→104，四任务平均得分线性提升；104 套模型≈等同在测试屋直接训练的 “in-domain” 基线。  
  3. **消融验证**  
    * 去掉 ME 或 CE → 成功率降 10–25 %；同时移除两者降至 40 % 以下。  
    * 去掉 WD，语言挑物任务在 OOD 物体上成功率由 62 %→38 %。  
    * 去掉 VI，高层推理失配，长任务成功率降低 ≈15 %。  
  4. **对比**  
    * π0-FAST+Flow：仅动作数据训练，平均 57 %；  
    * π0 原版：46 %；  
    * π0.5：75 %。  
  5. **速度**：VLM+动作专家推理 50 Hz，端到端无规划。  


  ### 关键词  
  开世界泛化；多模态协同训练；Vision-Language-Action (VLA)；高层子任务预测；Flow-Matching 动作专家；FAST Tokenizer；跨机体学习；家庭移动操作；长时序任务

---

- Learning to Act Anywhere with Task-centric Latent Actions 
*Y. Y. Qingwen Bu, Jisong Cai, Shenyuan Gao, Guanghui Ren, Maoqing Yao, Ping Luo, and Hongyang Li; Robotics: Science and Systems Conference 2025; DOI: 10.48550/arxiv.2505.06111*

  **想解决的问题**

  本文旨在解决现有机器人学习方法在跨环境、跨实体（embodiments）任务中存在的局限性。传统的机器人学习方法往往依赖于带有动作标注的大规模数据，这限制了它们在不同物理配置和环境中的泛化能力，难以学习可在不同机器人和任务间迁移的知识。因此，作者提出了一个新的框架，旨在从不同实体和视角的视频中提取任务中心（task-centric）的潜在动作表示，从而实现统一的视觉-语言-动作（VLA）策略学习，提升机器人在多种环境和任务中的执行能力。

  **具体做法**

  *算法部分：*

  1. **任务中心的潜在动作学习**

    - **潜在动作量化**：作者提出了一种基于VQ-VAE的潜在动作模型，通过在连续视频帧之间提取逆动力学，从而在无监督的情况下从视频中获取动作表示。具体而言，给定一对相隔固定时间间隔的视频帧 \( \{o_t, o_{t+k}\} \)，使用编码器 \( I(a_t | o_t, o_{t+k}) \) 来推断潜在动作 \( a_t \)。为了将潜在动作离散化，采用了矢量量化（vector quantization）方法，引入了一个代码簿（codebook），将连续的动作表示映射为离散的动作符号。

    - **任务相关与任务无关动态解耦**：为了消除任务无关的动态（例如相机抖动、环境中其他物体的运动）对潜在动作表示的影响，作者引入了任务指令（language instructions）作为条件，构建了一个两阶段的训练框架。

      - **阶段一**：通过在编码器和解码器中引入任务指令，模型学习到的潜在动作主要包含任务无关的动态信息，而任务相关的信息由任务指令提供。

      - **阶段二**：在固定阶段一中学习到的任务无关潜在动作的情况下，引入新的潜在动作 \( a^{TC} \) 去捕捉任务相关的动态。这样，通过代码簿的分离，模型能够更好地学习到与任务相关的潜在动作表示。

    - **利用DINO特征空间**：为了避免像素级预测带来的噪声，作者利用预训练的DINOv2模型提取的特征空间作为输入和预测目标。这些特征具有空间和对象中心的性质，有助于模型更好地捕捉任务相关的信息。

  2. **通用策略的预训练**

    - **基于Prismatic-7B模型**：作者使用了Prismatic-7B视觉-语言模型作为基础，结合了视觉编码器（SigLip和DINOv2）和LLaMA-2语言模型。

    - **将潜在动作作为特定的词汇**：在语言模型的词汇表中添加了特定的动作标记（例如ACT_1, ACT_2,...），将离散化的潜在动作映射到这些标记上，使模型能够在统一的潜在动作空间中进行规划。

    - **自回归的潜在动作预测**：模型接收观测 \( o_t \)、任务指令 \( l \) 和之前的潜在动作序列 \( a_{z,<i} \)，通过最小化下一个潜在动作的负对数似然来训练。

  3. **策略部署与下游适应**

    - **潜在动作解码**：为了将通用策略应用于具体的机器人，作者设计了一个轻量级的动作解码器，将模型生成的潜在动作解码为具体的机器人控制信号。这个解码器利用视觉特征和潜在动作特征，通过多头注意力机制，将潜在动作映射为可执行的动作序列。

    - **历史动作的利用**：在推理过程中，模型在每个时间步将之前的潜在动作作为输入的一部分，这类似于大型语言模型中的链式思维（Chain-of-Thought）方法，能够提升模型在长时序任务中的表现。

  *系统（设计）部分：*

  - **数据集与训练细节**

    - **大规模多样化的数据集**：为了训练通用策略，作者使用了多种源的数据，包括不同机器人的操作视频、人类的第一人称视频（如Ego4D数据集）等。

    - **训练过程的高效性**：由于引入了任务中心的潜在动作表示，模型在预训练过程中只需要较少的计算资源（OpenVLA的1/20），并且在下游任务中只需要较少的数据（OpenVLA的1/10）就能达到优异的表现。

  - **模型架构**

    - **视觉-语言-动作模型**：使用了融合视觉编码器（SigLip和DINOv2）和语言模型（LLaMA-2）的Prismatic-7B模型，能够处理视觉输入和语言指令，并预测潜在动作序列。

    - **轻量级动作解码器**：为了适应不同的机器人实体，设计了一个仅有10.8M参数的动作解码器，能够高效地将潜在动作解码为具体机器人的控制信号，支持异构的动作空间。

  **效果**

  - **在多种操纵和导航基准上的优异表现**：UniVLA在多个操纵任务基准（如LIBERO、CALVIN等）和导航任务（如R2R）上达到了最先进的性能，相比OpenVLA取得了显著的性能提升。

  - **高效的训练与适应**：UniVLA在预训练时仅使用了OpenVLA 1/20的计算资源，在下游任务中仅使用了OpenVLA 1/10的数据量，实现了更高的效率。

  - **跨环境与实体的泛化能力**：通过在统一的任务中心潜在动作空间中进行规划，UniVLA能够从不同的实体和环境中学习可迁移的知识，支持在人类视频等异构数据上进行预训练，并将其应用于不同的机器人。

  - **实验验证**：在真实机器人部署中，UniVLA在多个任务中表现出色，实现了36.7%的成功率提升，以及在多个环境和任务中的连续性能提升，证明了其在真实世界中的实用性。

  **关键词**

  - 任务中心的潜在动作表示, 视觉-语言-动作模型（VLA）, 跨实体学习（Cross-embodiment Learning）, 矢量量化（Vector Quantization）, 无监督学习, DINOv2特征空间, 通用策略, 逆动力学模型, 多模态数据, 行为克隆（Behavioral Cloning）

---

- Surgical Action Planning with Large Language Models
*Z. H. Mengya Xu, Jie Zhang, Xiaofan Zhang, and Qi Dou; ArXiv 2025; DOI: 10.48550/arxiv.2503.18296; https://dx.doi.org/10.48550/arxiv.2503.18296*

  ### 想解决的问题  
  * 机器人辅助手术（RMIS）现有智能模型多做事后分析（阶段/器械识别等），缺乏“面向未来”的术中预测与决策支持。  
  * 手术动作规划（SAP）需根据实时视频和手术目标生成下一步乃至长时序动作链，挑战包括：  
    1. 复杂器械-组织交互理解与时序依赖。  
    2. 大模型无法直接处理长视觉历史，导致上下文丢失。  
    3. 医疗数据隐私与标注匮乏，使得端到端微调成本高。  


  ### 具体做法  

  #### 1. LLM-SAP 框架  
  ```
  视觉历史 H = {v1…vt} + 目标描述 G
          ↓  Near-History Focus Memory (NHFM)
            ├─ DirNHFM: VLM 直接处理近帧 ⟨vt,at⟩ + 远程动作序列 {ai}
            └─ IndirNHFM: VLM 先产生日志 caption Ct → 供纯 LLM 使用
          ↓  Prompts Factory 生成两类提示
            · DCPrompts：帧级结构化描述
            · APPrompts：基于知识库的“进展评估+安全考量+三步动作排序”请求
          ↓  LLM / VLM-LLM 解码
  输出：下一动作预测 at+1 及可解释文本
  ```

  #### 2. 近历史聚焦记忆模块 (NHF-MM)  
  * 远程历史：仅用动作标签压缩表示，防止长序列干扰。  
  * 近历史：包含当前关键帧图像/描述 + 最近动作，提供细粒度上下文。  

  #### 3. 双模态适配  
  * **DirNHFM**：Qwen2-VL 72B 等多模态大模型，直接输入图像。  
  * **IndirNHFM**：普通 LLM（Qwen2.5 32B/72B）只能吃文本，因此先由 VLM 生成 Ct。  

  #### 4. Prompt Engineering  
  * **知识库**：手术流程、操作原则、安全守则、动作描述四大块嵌入。  
  * **APPrompts**：要求模型输出  
    1) Progress Assessment  
    2) Safety Considerations  
    3) Ready-to-Execute Actions（排序+理由）。  

  #### 5. 微调策略  
  * 零样本评估：直接提示工程。  
  * SFT+LoRA：用 GPT-4o 蒸馏生成 118 例高质量对话，8×A800   lr 1e-4, epoch 50。  

  #### 6. 数据集与指标  
  * **CholecT50-SAP**：自建，50 例 LC 视频 → 225 样本（5 类动作）。  
  * 指标  
    - SLAcc / VLAcc（Top-k）。  
    - **Relaxed Accuracy** (ReAcc)：预测在当前或下一帧出现即算正确，更贴合手术动态。  


  ### 效果  
  1. **零样本**  
    * Qwen2.5-72B (IndirNHFM) Top-1 SLAcc 45.6%，比 Qwen2-VL 提高 5.3%。  
  2. **监督微调**  
    * Qwen2.5-32B-SFT 标准 Top-2 SLAcc 78.9%，比未调优版提升 19.3%，ReAcc Top-2 达 95.3%。  
  3. **记忆策略消融**  
    * 仅近帧 vs. 近帧+远程动作：后者 Top-1 SLAcc 提升 ~7%。  
  4. 框架可输出“进展评估+安全建议+动作排序”文本，适用术前教学、术中导航、术后分析等场景。  


  ### 关键词  
  手术动作规划；大型语言模型；视觉语言模型；近历史记忆；Prompt 工程；LoRA 微调；零样本推理；Relaxed Accuracy


---

- Multimodal Robotic Surgical Instrument Transfer and Sorting Platform: Scrub Nurse Robot
*Y. H. Wing Yin Ng, Ke Xie, Philip Wai Yan Chiu, and Zheng Li; IEEE International Conference on Robotics and Biomimetics 2024*

  ### 想解决的问题  
  1. 手术器械的术中递送与术后整理高度重复、劳动强度大，易因疲劳导致数目记录或递拿错误。  
  2. 器械锐利且常带血污，人工反复抓握存在职业伤害风险。  
  3. 现有机器人方案多依赖条码、预设模板或固定摆位，遇到血污遮挡、姿态变化、器械交叠即失效；且缺乏与外科医生自然交互（语音）的完整闭环。  

  ### 具体做法  

  #### 1. 数据集构建  
  * **MIS-Instrument-Ex-vivo 数据集**  
    * 9 类、共 13 型胸腔镜器械，1 037 张 640×480 RGB 图。  
    * 标注：  
      * Oriented Bounding Box（OBB）：长边平行器械轴线，提供精确朝向。  
      * Occlusion 标签：Top / Bottom（被遮挡层次）。  
    * 两级子集：原图+OBB 用于检测；裁剪图+遮挡标签用于分类。  

  #### 2. 视觉双阶段模型  
  1. **器械检测与姿态**  
    * **YOLOv7-OBB**（改进头部及损失函数）  
      * 输出 7 维：xc, yc, L_long, L_short, confidence, class_obj, class_angle(0-179)。  
      * 损失 = 物体类别 + 角度分类 + OBB 回归 + OBB IoU 置信度。  
      * 结果：Precision 95.6%，Recall 97.4%，优于 YOLOv5-OBB（+9.2% AR）。  
  2. **遮挡推理**  
    * ResNet-50 二分类（Top/Bottom），输入为 OBB 裁剪后旋正 224×224 图。  
    * 数据增强：翻转、旋转、mosaic、mixup。  
    * 准确率 92%。  

  #### 3. 语音理解与人机交互  
  * **Silero ASR** 将外科口语转为指令，如 “Give me the stapler”。  
  * 预设词典映射至器械类别。  

  #### 4. 机器人系统集成  
  * **硬件**  
    * Franka Emika 7-DOF 机械臂（1 kHz 速度接口）  
    * 气动抓手 + 自研仿生指甲  
    * Intel RealSense D435i RGB-D 相机  
  * **坐标标定与运动**  
    * EasyHandEye 校准得到  ^{0}T_{c} ，抓取点取 OBB 中点，深度取 9 像素均值+5 帧平滑。  
    * MoveIt 规划笛卡尔路径，抓手气压经 Arduino 控制。  
  * **任务逻辑**  
    1. 视觉检测 → 器械列表{type, pose, depth, occ}.  
    2. 语音命令匹配目标器械。  
    3. 若目标为 Bottom，则先递归移除其上层 Top 器械。  
    4. 机械臂执行抓取、递送、返回。  
    * 伪代码见 Algorithm 1（文中）。  

  ### 效果  
  * 视觉：YOLOv7-OBB 在自建数据集上 mAP95.6%，ResNet-50 遮挡分类 92%。  
  * 物理实验：机器人可在多器械交叠、任意朝向情况下，按照语音指令自动选取可抓取目标并正确递送给外科医生。  
  * 视频展示证明平台鲁棒性与实时性（RGBD 30 Hz，整链响应 <2 s）。  

  ### 关键词  
  手术器械递送；机器人洗手护士；多模态人机交互；OBB 检测；遮挡推理；语音-视觉融合；Franka 机械臂






















## 8. Agent - 智能体 <a id="Agent"></a>

- RoboOS: A Hierarchical Embodied Framework for Cross-Embodiment and Multi-Agent Collaboration
*H. Tan, X. H. , C. C. , M. L. , Yaoxu Lyu, M. Cao, D. L. , et al.; 2025; DOI: 10.48550/arxiv.2505.03673; https://dx.doi.org/10.48550/arxiv.2505.03673*

  ### 想解决的问题  
  * 现有端到端 VLA 或分层 VLA 系统主要面向单机器人，存在：  
    1. 跨机体（single-arm / dual-arm / wheeled / humanoid）迁移困难，重复训练昂贵；  
    2. 多机器人并行协作缺乏统一调度与实时共享记忆，长任务易出错；  
    3. 仅靠单一模型难以同时满足“高层认知规划 + 低层毫秒级执行”的时延要求；  
    4. 边-云协同与大模型推理成本高，缺少可扩展部署框架。  


  ### 具体做法  

  #### 1. 整体架构：Brain-Cerebellum 双层  
  1. Embodied Brain Model（RoboBrain-1.5-OS，云端）  
  * 基座：Qwen2.5-VL-7B → 三阶段增量训练  
      - Stage-1：3M 通用 VLM 数据，奠定视觉-语言基础  
      - Stage-2：2.3M 机器人规划/指点/可承载区域/轨迹四大能力数据  
      - Stage-3：RoboOS-Enhanced 数据：  
        · Multi-Robot-45k（场景图+多机器人工作流）  
        · Robotic-Agent-144k（正负 Observation-Action 对，支持 RL GRPO）  
  * 产出：  
      - 多机器人任务分解 & DAG 工作流生成  
      - Agent-Tool 调用序列  
      - 持续三层反馈（执行-感知-记忆）纠错  

  2. Cerebellum Skill Library（边端）  
  * Manipulation：  
      - VLA-tools：OpenVLA、RDT-1B、π0、Octo  
      - Expert-tools：经典抓取/可承载抓取、接触力控制  
  * Navigation：SLAM、MapNav、VLN-based  
  * Specialized：接触丰富、变形物、灵巧手等  
    * 标准化 Robot Profile，支持 plug-and-play；每个子技能通过 gRPC/WebSocket 暴露，使 Brain 可动态装载。  

  3. Real-Time Shared Memory（Redis+SceneGraph）  
    * Spatial：层级场景图（楼层-房间-对象）SAM+CLIP 聚合；维护拓扑/支持查询与更新  
    * Temporal：任务历史、Tool 调用反馈、异常日志  
    * Robotic：各机器人运动域、关节状态、电量；支持负载均衡、故障预测  
    * 发布-订阅 <1 ms 延迟，实现多机体状态同步  

  #### 2. 工作流管线  
  Step-1 全局任务解析 → Brain 调用 RAG(场景记忆+说明) 生成 reasoning trace R 与 subtask graph G  
  Step-2 Monitor 根据 DAG 深度并结合机器状态并行/串行分派 (d, robot_id)  
  Step-3 每个 Subtask 生成 Agent，按观察-工具序列闭环执行，失败时自恢复（基于记忆）  
  Step-4 Shared Memory 增量更新场景/反馈/机器人记忆，触发 Brain 再规划  

  #### 3. 边-云推理优化（FlagScale 框架）  
  * Zero-3 并行 + Tensor Parallel；FP16、W8A16 量化  
  * RTX4090 单卡低层推理 7 ms；云端多 A800 集群推理 20 ms 级  
  * 双向心跳与镜像缓存，保证>50 Hz 机器人控制环  

  ### 效果  
  1. Embodied 能力基准（表 1）：  
    * 多机器人规划 AR 提升 28%（对比 Qwen2.5-VL），超越 DeepSeek-V3-685B 5.5%  
    * Affordance mAP 提升 16.9%，Trajectory 误差 DFD / HD / RMSE 分别降 40%+  
  2. 三场景真实验证  
    * 餐厅：Unitree Humanoid + AgileX 双臂 制作并上菜；  
    * 住宅：RealMan 单臂 + AgileX 双臂 递送水果刀与橙子；  
    * 超市：双臂开礼袋、单臂挑礼物，多机器人并行完成 >10 子任务，零死锁。  
  3. 性能  
    * Redis 共享内存同步 <1 ms；多 GPU FP16 推理批量加速 48–63%  
    * 任务并发管理多 DAG，动态调度可支持 >20 机器人实例  

  ### 关键词  
  跨机体协作；Brain-Cerebellum 分层；多机器人任务分解；实时共享记忆；RoboBrain-1.5-OS；Cerebellum Skill Library；FlagScale 边-云推理；Multi-Robot-45k；Agent-Based Tool Invocation

---

- Hi Robot: Open-Ended Instruction Following with Hierarchical Vision-Language-Action Models
*B. I. Lucy Xiaoyang Shi, Michael Equi, Liyiming Ke, Karl Pertsch, Quan Vuong, James Tanner, Anna Walling, Haohuan Wang, Niccolo Fusai, Adrian Li-Bell, Danny Driess, Lachy Groom, Sergey Levine, Chelsea Finn, Lachy Groom, Sergey Levine, Chelsea Finn; ICML 2025 2025*

  ### 想解决的问题  
  1. 机器人在真实开放环境中需处理长时序、多步骤任务，但现有 VLA 端到端策略只能解析简单原子指令，无法理解“复合提示+实时纠正”这类人机互动。  
  2. 直接依赖大型通用 VLM/LLM（如 GPT-4o）缺少对机器人可行动作的物理 grounding，容易生成不可执行或与场景不符的指令。  
  3. 缺乏一种能够在**高层语义推理**与**低层物理控制**间无缝衔接、同时可大规模自动化构造训练数据的框架。  


  ### 具体做法  

  #### A. 分层架构  
  1. 高层策略 (System-2)：基于 PaliGemma-3B 微调的 VLM  
    • 输入：多视角 RGB 图像 + 用户自然语言（含反馈）  
    • 输出：a) 机器人回应文本 *uₜ*（语音播报） b) 低层可执行原子指令 *ĉₜ*（如 “pick up bowl”）。  
  2. 低层策略 (System-1)：采用 π0 Vision-Language-Action (VLA)  
    • 以 *ĉₜ*、图像、机器人关节状态为条件，Flow-Matching 动作专家连续生成 50-step action chunk（50 Hz），直接驱动 6-DoF 单/双臂 + 移动底盘。  
  3. 推理频率  
    • 高层 1 Hz（或检测到新语音立即触发）；低层 50 Hz。  

  #### B. 数据与训练  
  1. 实演数据 D_demo：人工遥操作收集，按技能切分并粗标原子指令。  
  2. **合成交互数据 D_syn**（核心创新）  
    • 用大 VLM 作为 *Data-Generator*：给定当前图像 + 技能标签 → 生成可能的用户提示 ℓₜ、机器人回应 uₜ，实现“虚拟人机对话”。  
    • 覆盖场景分类：否定约束、实时纠正、偏好约束、任务拆分等。  
  3. 训练流程  
    ① 高层：在 D_labeled∪D_syn 上做 next-token 交叉熵训练，学习 p_hi(ĉₜ,uₜ | I,ℓ)。  
    ② 低层：在 D_demo∪D_labeled 上用 FAST 离散 token 预训练，再引入 Flow-Matching 动作专家微调。  

  #### C. 系统集成  
  • 语音接口：Whisper large-v2 本地转写 + TTS 输出。  
  • GPU 推理：RTX 4090 单卡高层 60 ms、低层单步 7 ms（10 步并行），满足实时。  
  • 平台：UR5e 单臂、双臂 ARX、移动 ARX（三种 DoF: 7/14/16）。  

  ### 效果  
  1. 三大任务场景  
    • Table Bussing：根据提示仅清理垃圾/仅收餐具/仅黄色物体，并处理中途“那不是垃圾”等口头纠正。  
    • Sandwich Making：遵循“无番茄”“乳糖不耐”等复杂夹层组合与实时补充/取消配料。  
    • Grocery Shopping：移动双臂抓取多件零食饮料入篮并搬运。  
  2. 定量指标（20 trial/任务）  
    • Instruction Accuracy 提示对齐率：Hi Robot 77%，GPT-4o 33%，Flat VLA 31%。  
    • Task Progress 任务完成度：Hi Robot 平均 68%，GPT-4o 29%，Flat VLA 25%。  
  3. 消融  
    • 去掉合成数据 → 指令对齐下降 39%，无法处理用户偏好与纠正。  
    • Flat VLA +合成数据 → 无层次仍比 Hi Robot 低 19%，说明推理层级的重要性。  
  4. 人类高层 Oracle 上限 78%/72%，Hi Robot 已逼近。  
  5. 推理示例：GPT-4o 出现 “pick up bermuda triangle”等幻觉；Hi Robot 能识别被抓住的碗非垃圾并放回。  


  ### 关键词  
  层次化 Vision-Language-Action；高层语义推理；低层 Flow-Matching 控制；合成语言交互数据；实时用户纠正；开放场景指令跟随；PaliGemma；π0 VLA；人机协作

---

- CLEA: Closed-Loop Embodied Agent for Enhancing Task Execution in Dynamic Environments 
*Mingcong Lei, G. W., Yiming Zhao, Zhixin Mai, Qing Zhao, Yao Guo, Zhen Li, Shuguang Cui, Yatong Han, and Jinke Ren; IEEE International Conference on Robotics and Automation 2025; DOI: 10.48550/arxiv.2503.00729*

  **想解决的问题**

  本文旨在解决大型语言模型（LLMs）在动态环境中用于机器人任务执行时面临的挑战。虽然LLMs在复杂任务的语义推理和分层分解方面具有显著能力，但在嵌入式系统中应用时，确保子任务序列的可靠执行和在长期任务中一次性成功存在困难。具体问题包括：

  1. **缺乏适应性的任务规划**：LLMs生成的动作序列是静态的，无法适应动态环境中对象状态和空间关系的变化，导致在真实世界应用中的鲁棒性不足。

  2. **有限的上下文窗口**：LLMs在处理长期任务时，受限于其上下文窗口大小，无法持续跟踪任务状态，影响了任务执行的连续性。

  3. **部分可观测环境中的决策不确定性**：机器人只能通过自身的传感器获得部分环境信息，需要在不完全了解环境状态的情况下进行决策，这增加了任务执行的复杂性。

  **具体做法**

  本文提出了一个名为 **CLEA(Closed-Loop Embodied Agent)** 的闭环框架，通过集成四个专门的开源LLM模块，实现了在动态、多机器人环境中自适应的任务规划和执行。框架的核心创新包括：

  *算法部分：*

  1. **闭环规划-评估架构**

    - **规划器（Planner）**：采用**层次化规划机制**，基于环境的置信状态和可用信息，**动态生成可执行的子目标和对应的动作序列**。规划器利用链式思维（Chain-of-Thought，CoT）提示，通过预定义的技能池生成结构化的动作序列，确保动作序列与机器人实际能力相匹配。

    - **评估器（Critic）**：在每个动作执行前，**实时评估**规划器生成的动作的可行性。评估器采用视觉语言模型（VLM）协同CoT提示，结合当前环境的视觉观察和置信状态，**判断动作是否适合当前环境状态**。如果动作不可行，评估器生成反馈信号，触发规划器重新规划。

  2. **环境记忆与置信状态估计**

    - **观察者（Observer）**：利用VLM将机器人摄像头捕获的视觉输入**转换为结构化的文本描述**，提取任务相关的对象和空间关系，为后续的规划和评估提供可靠的环境信息。

    - **记忆模块（Memory）**：包括**历史缓冲区**和**摘要器**。历史缓冲区记录机器人与环境交互的历史信息，包括观察、动作和反馈。摘要器利用LLM的总结能力，**将历史交互信息压缩为置信状态（belief state）**，提供给规划器作为决策依据。

  3. **部分可观测马尔可夫决策过程（POMDP）建模**

    - 将机器人在动态环境中的任务执行建模为POMDP，考虑到机器人只能通过自身传感器获得部分环境信息，存在决策不确定性。CLEA通过迭代更新置信状态，**在不确定环境中实现闭环的感知-推理-执行**。

  *系统设计部分：*

  1. **模块化架构**

    - **四个解耦的LLM模块**分别承担不同的功能：观察者、摘要器、规划器和评估器。这样的解耦设计使得每个模块可以独立优化，增强了系统的可扩展性和鲁棒性。

  2. **开源模型的应用**

    - 为了增强可复现性和可扩展性，CLEA使用了开源的VLM和LLM模型。具体而言，LLM模块采用了**Qwen2.5-72B-Instruct模型**，VLM模块采用了**Qwen2.5-72B-VL-Instruct模型**。

  3. **可执行的技能池**

    - 定义了**预定义的技能池**，包含了机器人可以执行的基本动作（如打开、关闭、拾取、放置、导航等），这些动作映射到机器人底层的操作API。规划器生成的动作序列由这些技能组成，确保了动作的可执行性。

  4. **系统集成与实验设置**

    - 搭建了**真实环境下的实验平台**，包括一个双臂移动机器人和一个固定基座的单臂机器人。环境是一个小型开放式厨房，包含可操作的对象、容器和设备。
    - **感知系统**：融合了YOLOv11分割模型和基于粒子滤波的定位与地图构建技术，增强了场景理解和导航能力。
    - **运动规划**：在所有机器人平台上使用了基于MoveIt框架的RRT-Connect算法，实现了机器人运动规划和执行。

  **效果**

  - **实验验证**：在真实的厨房环境中进行，共设计了三种任务类型（搜索任务、操作任务、搜索-操作集成任务），每种任务进行了多次试验，总计12次实验。

  - **结果对比**：

    - 与开放式的基线模型相比，CLEA在成功率（SR）上提升了**67.3%**，在平均得分（AS）上提升了**53.8%**。
    - 在消融实验中，与缺少评估器模块的CLEA变体相比，完整的CLEA在成功率上提升了**42.0%**，在平均得分上提升了**26.3%**。

  - **性能分析**：

    - **鲁棒性增强**：CLEA能够在动态环境中，根据环境变化和任务执行情况，实时调整策略，避免了由于环境不确定性导致的任务失败。
    - **任务执行效率**：通过评估器的实时反馈，CLEA减少了冗余和无效的动作，提高了任务执行的效率。

  - **失败案例分析**：

    - **动作API限制**：由于预定义的动作格式限制，规划器有时无法生成最优的动作序列。
    - **评估器的限制**：评估器在少数情况下无法准确识别动作的可行性，导致错误的计划未被及时纠正。
    - **多机器人协作问题**：在需要多机器人协作的任务中，CLEA有时难以正确理解和管理机器人之间的交互关系。

  **关键词**

  - 闭环规划评估架构, 大型语言模型（LLM）, 视觉语言模型（VLM）, 部分可观测马尔可夫决策过程（POMDP）, 机器人任务执行, 动态环境, 多机器人协作, 计划-执行-评估循环, 环境记忆与置信状态, 开源模型应用

---

- RoboMemory: A Brain-inspired Multi-memory Agentic Framework for Lifelong Learning in Physical Embodied Systems
*M. Lei, H. C. , B. Q. , Zezhou Cui, Liangchen Tan, J. Hong, G. H. , Shuangyu Zhu, Yimou Wu, S. J. , Ge Wang, et al.; 2025*

  ### 想解决的问题  
  1. 当今基于 VLM/LLM 的 embodied-agent 往往聚焦单任务或短时操作，缺乏跨任务、跨场景的“终身学习”能力。  
  2. 现有记忆框架要么过于简化（仅短期缓存），要么多模块串行导致推理延迟高，难以在真实机器人中落地。  
  3. 长时规划中的死循环、动作失配等问题缺少一套“规划-执行-记忆”闭环机制来检测并自愈。  

  ### 具体做法  

  #### 1. 整体脑启发式架构  
  ```
  信息预处理(丘脑)  →  终身记忆系统(海马)  →  规划/批判器(前额叶)  →  低层执行(小脑+VL A)
          │                         ↑↓(并行RAG)           │
          └──视频/语音/触觉→ 文本摘要、查询           行动反馈 → Critic
  ```

  #### 2. 三层多模态记忆体系  
  A. 工作记忆（Working）: action 级别 FIFO 缓冲，保存最近 N 步摘要。  
  B. 短期记忆（STM, Mixed）: 任务过程中持续更新的时空图 + 失败/成功标签。  
  C. 长期记忆（LTM, Task level）:  
    • Spatial–KG：动态检索式增量更新算法  
      - 采用 K-hop 子图检索 + 局部冲突检测合并，复杂度 O(D·K)；理论界定见定理1/2。  
    • Temporal：FIFO+摘要压缩，保留行动序列。  
    • Episodic：任务级交互日志。  
    • Semantic：动作-结果经验向量库，用作技能评估。  

  #### 3. 统一“并行更新/检索”范式  
  四子模块共享：`Update(new_info) → Retrieve(query)` 接口；并行执行避免级联延迟。  
  RAG 检索策略：  
  ```
  K' = embed(指令+初始场景) → Top-k 经验 → Planner prompt
  ```

  #### 4. Planner–Critic 闭环  
  1. Planner(VLM) 输出多步计划。  
  2. Critic 在每步执行前评估场景/记忆，若不一致则触发重规划；为避免死循环，首次动作不经 Critic。  
  3. 行动通过 π0 或 LoRA-fine-tuned VLA 转为连续控制；失败反馈写入 STM→LTM。  

  #### 5. 执行层与硬件适配  
  * 模拟：EB-ALFRED/Habitat 使用高层 API。  
  * 真实：Mobile-ALOHA 双臂 + SLAM；自采 1,040 episode LoRA 微调 π0，支持 pick/place/nav/open 等十类高层指令。  

  ### 效果  

  1. **基准测试**  
    * EB-ALFRED(Base+Long, 100 任务)：  
      - RoboMemory(Qwen2.5-VL-72B) SR 67% / GC 78.4%  
      - 高出开源底座(+25%)，超越闭源 Claude-3.5 Sonnet 5%。  
    * EB-Habitat：平均 SR 提升 24%，GC 提升 12%。  

  2. **消融**  
    * 去 critic → SR-12%  
    * 去 Spatial KG → SR-20%  
    * 去 LTM → Long 集合 SR-18%  

  3. **真实厨房 15 任务两轮试验**  
    * 第一次平均成功 46%；记忆保留后第二轮升至 73%，证明终身学习能力。  

  4. **效率**  
    * 并行 KG 更新仅处理相关子图，推理延迟控制在 <100 ms；整体系统可 10 Hz 闭环。  

  ### 关键词  
  终身学习；多模态记忆；动态知识图谱；Planner-Critic 闭环；RAG 检索；跨任务泛化；机器人多记忆框架；VLA 执行；EmbodiedBench

---

- Experience is the Best Teacher: Grounding VLMs for Robotics through Self-Generated Memory
*K. Q. Guowei Lan, Rene Zurbr ´ ugg ¨, Changan Chen, Christopher E. Mower, Haitham Bou-Ammar, Marco Hutter; 2025; DOI: 10.48550/arxiv.2507.16713; https://dx.doi.org/10.48550/arxiv.2507.16713*

  ### 想解决的问题  
  * 现有视觉语言模型（VLM）虽能进行机器人任务规划，但因仅在互联网数据上训练，**不了解具体机器人自身能力与局限**，导致在真实场景中常输出不可执行或高失败率的计划。  
  * 纯靠人工标注或外部监督对不同机器人做“能力对齐”代价高、效率低。  
  * 缺乏一种能让机器人**自主生成经验、反思失败并将经验固化为长期记忆**的方法，从而持续提升跨任务、跨场景的成功率。  


  ### 具体做法  

  #### 1. 框架总览 EXPTEACH  
  ```
  用户指令 I + 初始观测 o0
          │
    场景描述器(同VLM) → 场景Key K'
          │
    RAG检索长期记忆LTMs {Ki,Ei}
          │
  短期记忆STM m  ← 反思反馈
          │
        任务规划器 T (VLM) ——→ 动作 at
                                │
                        机器人执行
                                │
                    新观测 ot+1 → 成功检测器 D (同VLM)
  ```

  #### 2. 核心组件  
  1. **VLM 双角色**  
    * Planner `T`：根据 I、当前图像、STM、检索到的 LTM 生成下一动作（函数调用格式），支持 pick/place/push，并可触发“图像标注工具”。  
    * Detector `D`：执行后评估成功与否，返回 scene 描述、成功状态、失败原因、下一步建议。  
    ✔ 使用同一个 GPT-4o 级别 VLM 权重，减少模型切换。

  2. **Short-Term Memory (STM)**  
    * 格式：`m = {(aτ , rτ+1)}_{τ=0}^{t-1}`。  
    * 作用：  
      - 记录 **动作-反馈链**，提供反思信息。  
      - 失败时可触发**交互式重规划**（如先移除障碍物、使用替代工具）。  
    * 体现：20 s 任务内反复调用；人类只在场景被破坏时重置环境。  

  3. **Long-Term Memory (LTM)**  
    * 生成：任务完成后由 **Experience Summarizer** 将 STM 摘要化为 `(Key, Experience)` 并存储。  
      - Key = 场景描述器生成的“指令 + 初始场景”文本。  
    * 检索：新任务时用 `text-embedding-3-large` 生成向量，**RAG** 取余弦相似度 Top-k 经验，送入 Planner 作为上下文。  
    * 规模：实验共 100 条（96 条简单任务 + 4 条复杂任务生成）。  

  4. **On-Demand 图像标注工具**  
    * 触发条件：Planner 判断 grasp/place/push 需精准位置或特定抓取部位 → 请求标注。  
    * 流程：Grounded-SAM 对目标生成 mask →   FPS 或可视编号标注 → VLM 选取最佳区域。  
    * grasp 再经过 AnyGrasp + IK 过滤，最终以 `g* = argmax s_conf · s_loc` 输出抓姿。  

  ### 效果  
  1. **短期记忆 + 反思**  
    * 4 个高难度操控任务（每任务 5-10 trial，允许 2 次尝试）：平均成功率由 36% → **84%**。  
    * 出现“创意工具使用”：用海绵推糖果、用推方式搬 fragile egg 等。  

  2. **长期记忆 Grounding**  
    * 12 个场景（含 8 个未见过）一次尝试成功率从 22% → **80%**。  
    * 指令“Pick up milk carton (apple on top)”立即检索到“先移走顶上物再取容器”的经验，一次成功。  

  3. **检索策略消融**  
    * Random-k 记忆：27%  
    * 全量 LTM：67%（噪声干扰）  
    * **RAG Top-k：89%**  

  4. **图像标注消融**  
    * 需特定抓取部位的 7 类物体 grasp 成功率平均提高 30-60%；  
    * 6 类 push 任务到目标误差平均减少 25-40%。  


  ### 关键词  
  自生成记忆；短期记忆反思；长期记忆检索；RAG 机器人对齐；VLM 任务规划；图像标注 Grounded-SAM；工具使用涌现；跨场景泛化











## 9. Medical Robot(Surgical robot) - 医疗和手术相关机器人 <a id="Medical_Robot"></a>

- Real-Time Pose Detection for Magnetic Medical Devices
*M. B. Christian Di Natali, and Pietro Valdastri; IEEE TRANSACTIONS ON MAGNETICS 2013 Vol. 49*

  ### 想解决的问题  
  * 为实现永磁体磁驱胶囊内镜的闭环控制，必须实时获得胶囊 6 DOF 位姿。  
  * 现有常用定位技术（电磁跟踪、光学、交变磁场三角定位等）在强永磁场下易受干扰或仅能输出 3 DOF，无法满足磁驱操作要求。  
  * 需要一种可在强静磁场中工作的、尺寸与功耗均可嵌入胶囊的实时 6 DOF 定位方法。  

  ### 具体做法  

  #### 1. 传感硬件与布局  
  1. 胶囊内部磁体：NdFeB，φ 11 mm × 11 mm，轴向充磁。  
  2. 传感器  
    * 6 × 单轴 Hall（CYP15A，量程 0.1–2 T）——在胶囊两端沿 轴对称布置，形成两个三轴磁场测点 B₁、B₂，基线距 12 mm。  
    * 1 × 三轴加速度计（LIS331DLH）——实时给出俯仰 θ、偏航 φ。  
  3. 数据采集：16 bit DAQ 6211（20 kS/s）；方案可替换为 CC2530 + ADS8320 以实现体内无线采集。  

  #### 2. 磁场数值地图（离线）  
  * 外永磁体（EPM）：NdFeB N52，φ 50 mm × 50 mm，轴向充磁。  
  * 使用磁流模型 (Magnetic Current Model) 对 EPM 求解，推导圆柱坐标闭式积分式：  
    \( \mathbf{B}(\rho,z)=\mathbf{\hat\rho}B_\rho(\rho,z)+\mathbf{\hat z}B_z(\rho,z) \) ，详见式(4)。  
  * 在 30 cm × 30 cm × 30 cm 工作区离散 0.2 mm 网格；利用轴对称性仅保留 y–z 平面 15 cm × 15 cm 切片，显著减小存储。  

  #### 3. 实时迭代算法（Matlab 实现）  
  1. **初始标定**：将胶囊放在已知姿态下测一次得到 roll₀ 作为基准（俯仰/偏航由加速度直接得出）。  
  2. **坐标转换**  
    * 取得两组磁场测量 \( \mathbf{B}_1,\mathbf{B}_2 \)。  
    * 按当前俯仰 θ、偏航 φ、迭代得到的滚转 ψ，构造旋转矩阵  
      \( R_{w}^{c}=R_z(ψ)\,R_y(θ)\,R_x(φ) \) ，将传感器数据旋转到世界坐标，再用机器人正解得到与 EPM 座标系 \(\{E\}\) 的关系：  
      \( \mathbf{B}_i^{E} = R_{W}^{E} R_{w}^{c} \mathbf{B}_i \)。  
  3. **地图检索**  
    * 对每个 \( \mathbf{B}_i^{E} \) 转为圆柱坐标分量 \( (B_\rho,B_z) \)。  
    * 顺序遍历磁场切片查找最近邻点，得到对应位置向量 \( \mathbf{p}_i \)。  
  4. **位姿求解**  
    * 位置：胶囊中心 \( \mathbf{p}_c = (\mathbf{p}_1+\mathbf{p}_2)/2 \)。  
    * 滚转：计算 \( \mathbf{p}_1,\mathbf{p}_2 \) 在 x–y 平面的连线与 x 轴夹角。  
  5. **迭代更新**：用新 ψ 进入下一循环；采样周期 20 ms（运算 14 ms + 搜索 5 ms）。  

  #### 4. 实验平台  
  * RV-6SDL 六轴机械臂搬运 EPM；胶囊固定在三自由度万向架（不限制姿态）；Micron Tracker 光学系统提供旋转真值(0.3°)。  
  * 在 EPM 上方 10–15 cm、30 cm × 30 cm 平面，多层 z 方向扫描验证。  

  ### 效果  

  1. **定位精度（均值 ± σ）**  
    * 当胶囊–EPM 距离 ≤ 10 cm：  
      * 位置误差：x 1.4 ± 1.3 mm，y 2.2 ± 1.5 mm，z 1.7 ± 1.4 mm  
      * 滚转 ψ 误差 ≈ 1 °；俯仰/偏航由加速度计给出，误差 < 1 °  
    * 距离 15 cm 时平均误差仍 < 5 mm，但 σ 增大。  
  2. **成功率**  
    * 若允许 11 mm 球形误差容限（典型胶囊直径），10 cm 工作层定位成功率 100%，15 cm 层约 84%。  
  3. **实时性**  
    * 单周期 19 ms，可与机器人 50 Hz 控制环闭合。  
  4. **模块尺寸功耗**  
    * 传感单元 15 mm × 18 mm，功耗 < 15 mW，可集成于无线胶囊。  

  ### 关键词  
  磁驱胶囊内镜 Hall-IMU融合 磁场地图查表 6 DOF 实时定位 永磁体偶极模型

---

- Closed Loop Control of a Tethered Magnetic Capsule Endoscope
*A. Z. Taddese, P. R. Slawinski, K. L. Obstein and P. Valdastri; 2017; Publisher: Robotics: Science and Systems Foundation ; DOI: 10.15607/rss.2016.xii.018; https://dx.doi.org/10.15607/rss.2016.xii.018*

  ### 想解决的问题  
  1. 传统结肠镜“尾端推挤”导致黏膜应力大、患者不适，亟需前端牵引的磁驱软管镜以降低痛感。  
  2. 单纯人工操控外部永磁体难以精细定位，且胶囊受软缆摩擦、褶皱阻挡易失耦合，造成操作低效。  
  3. 现有磁驱系统多为开环或仅做 3-DoF 位置控制；缺乏结合实时 6-DoF 定位、可抵抗软缆扰动的闭环 4-DoF（平面位置+俯仰/偏航）控制框架。  

  ### 具体做法  

  #### 1. 硬件系统  
  * 6-DoF 机器人 (Mitsubishi RV-6SDL) 末端安装 4″×4″ NdFeB 永磁体（EPM）。  
  * 胶囊：φ20 mm×22 mm，内置小磁体 (11 mm)，6 轴 Hall + IMU，软缆承载摄像头、电线、灌注/工具通道。  
  * 实时采样 100 Hz，经细软缆传输至 ROS 控制 PC。  

  #### 2. 6-DoF 磁场定位  
  1. 外磁场查表：COMSOL 预离线计算 EPM 场，利用轴对称性存储 1/4 平面。  
  2. Hall 阵列测得外场 **b_c** → 旋转到 EPM 坐标系 **b_m**。  
  3. 在场图中快速顺序搜索得到胶囊位置 **p_c**（3-DoF）。  
  4. 扩展互补滤波 ECF 融合 IMU（俯仰/翻滚）+外场方向作偏航基准：  
    * g 向量 → 计算俯仰φ、翻滚θ  
    * 当胶囊靠近 EPM 奇异平面时，使用 ˆb_c × ˆb̄_c 误差补偿陀螺漂移，输出四元数 **q_c**。  
  → 获得 6-DoF 位姿，无需 X 射线或视觉。  

  #### 3. 磁力/力矩 – 机器人雅可比  
  * 偶极–偶极模型：  
    ```
    f_m = 3μ0|m_a||m_c|/4π|p|^4 (m̂_a m̂_c^T + m̂_c m̂_a^T + (m̂_c^T Z m̂_a)I) p̂
    τ_m = μ0|m_a||m_c|/4π|p|^3 m̂_c × D(p̂) m̂_a
    ```
    D = 3p̂p̂ᵀ−I , Z = I−5p̂p̂ᵀ  
  * 推导 ∂f/∂p 等九个偏导，构建 **JF**(p,m̂_a,m̂_c)。  
  * 将 EPM 几何雅可比 JA(q)（排除沿偶极轴旋转）嵌入，得 6×6 “力-力矩-关节”雅可比 **J_FA**。  

  #### 4. 加权阻尼最小二乘 + 梯度投影  
  ```
  q̇ = J⁺_FA (u_f - f_curr , u_τ - τ_curr)ᵀ  +  β(I-J⁺_FA J_FA) ∇g
  ```
  * 任务空间权重 Wx：增大扭矩分量权重，优先稳向；  
  * 关节权重 Wq：Liegeois 关节极限函数，保持机械臂肘上姿。  
  * 目标函数 g(q)：引导 5-6 轴保持临床友好“肘上”配置。  

  #### 5. 4-DoF PI 控制器  
  * 位置：tangent 速度误差 + lateral 偏差 → f_d。  
  * 姿态：heading 误差 **e_o** → τ_d。  
  * 维持垂向吸引力 0.45 N（安全压壁），Z 向运动未控制。  

  #### 6. 动力学仿真  
  * Gazebo+ODE，编写磁交互插件；软缆离散多段+万向节+阻尼模拟。  
  * 所有算法在 ROS 100 Hz 与实验平台无缝切换。  

  ### 效果  

  1. Gazebo 直线/正弦轨迹：  
    * 平面均方误差 1.1 mm / 5.0 mm；航向误差 0.11 rad。  
  2. 物理实验（水平双板+润滑油）：  
    * 直线：均误差 1.2 ± 1.4 mm，最大 9.6 mm。  
    * 正弦 A=55 mm λ=200 mm：均误差 10.3 ± 6.7 mm，最大 35.7 mm；平均航向误差 0.26 rad (15°)。  
    * 误差量级 < 胶囊直径（20 mm），适应 34–75 mm 结肠腔径。  
  3. 定位速率 100 Hz；整体闭环无磁耦丢失，可在软缆推拉扰动下快速恢复。  
  4. 系统维持 0.5 N 以下壁压，符合临床安全阈值，展示了磁驱疼痛减轻潜力。  

  ### 关键词  
  磁驱胶囊内镜；闭环控制；外永磁–内磁偶极；实时磁场定位；加权阻尼最小二乘；梯度冗余分解；Gazebo 仿真；结肠镜替代


---

- Enhanced Real-Time Pose Estimation for Closed-Loop Robotic Manipulation of Magnetically Actuated Capsule Endoscopes
*A. Z. Taddese, P. R. Slawinski, M. Pirotta, E. De Momi, K. L. Obstein and P. Valdastri; Int J Rob Res 2018 Vol. 37 Issue 8 Pages 890-911; Accession Number: 30150847 PMCID: PMC6108552 DOI: 10.1177/0278364918779132; https://www.ncbi.nlm.nih.gov/pubmed/30150847*

  ### 想解决的问题  
  1. 磁驱胶囊内镜闭环控制依赖 6-DOF 实时位姿估计，但单磁源定位存在两大瓶颈：  
    * 奇异面（磁偶极 m 垂直平面）区定位发散，恰是临床常驻工作区。  
    * 初始偏航角须人工精准校零，且运行中易漂移。  
  2. 现有单永磁 + Hall/IMU 方案在奇异区误差可达厘米级，无法支撑可靠闭环导航与自动化操作。  

  ### 具体做法  

  #### 1. 双磁场混合定位架构  
  * 在机器人末端永磁体（EPM，偶极 m_E）正交安装小型电磁线圈（m_C ⟂ m_E），线圈通 300 Hz PWM 方波产生微弱时变磁场。  
  * Hall 阵列 6 轴布置于胶囊，IMU 提供俯仰/翻滚。采样 18 kHz → Goertzel 滤波：  
    * 直流分量 → EPM 场 B_E  
    * 基波幅值 → 线圈场 B_C  
  * 两磁源在空间无共面奇异区，保证完整约束。  

  #### 2. 观测模型  
  ```
  Bs_E = R_w^s^T · R_z(γ) · R̃_w^s · B_E(p)
  Bs_C = R_w^s^T · R_z(γ) · R̃_w^s · B_C(p)
  ```
  其中 γ 为未知偏航漂移，R̃_w^s 由 IMU 解算。  
  磁场模型：采用 Derby-Olbert 圆柱永磁/线圈闭式解 + 查表加速（180 mm 线圈，160 匝）。  

  #### 3. 并行粒子滤波 6-DOF 状态估计  
  * 状态 x = [p_x p_y p_z γ]ᵀ，随机游走过程。  
  * 观测似然 p(z|x) 由 12 维磁场误差高斯化并加入归一化权重矩阵 W_z（三阶幅值差异）。  
  * 1×10⁴ 粒子 OpenMP 并行，100 Hz 更新；卷积核 Q = diag(1.5 mm², … ,0.01 rad²)。  
  * 粒子聚类球半径 η=0.1 提取鲁棒 MAP。  

  #### 4. 闭环磁驱控制示例  
  * 2D 位置 + 2D 姿态 PI 控制：  
    * 滑动面：tangent 速度误差+ lateral 位置误差；  
    * 线速度 5 mm/s，垂向力 0.45 N 限制。  
  * 永磁体–胶囊力矩雅可比 J_FE 线性化求逆产生关节速度指令（6-DOF 机器人）。  

  ### 效果  

  1. 静态螺旋实验（150–200 mm 半径）：  
    * 位置均方误差 ≤5 mm，姿态误差 φ,θ≈1°，ψ≈5°。  
    * 在 EPM 奇异面误差仍 <6 mm / 4°。  
  2. 与单源方法对比：在奇异区距离误差从 >40 mm 降至 <6 mm；无需初始偏航校准。  
  3. 动态跟踪（10–50 mm/s）：闭环横向稳态误差 <8 mm，平均航向误差 <7°。  
  4. 自动直线路径演示：胶囊受扰（手推软缆 40 mm）10 s 内恢复，维持 0.5 N 垂直安全力距。  

  ### 关键词  
  磁驱胶囊内镜；位姿估计；双磁场解耦；Hall-IMU 融合；粒子滤波；磁奇异性消除；闭环控制



---

- 6-D Spatial Localization of Wireless Magnetically Actuated Capsule Endoscopes Based on the Fusion of Hall Sensor Array and IMU
*Y. L. Heng Zhang, Zheng Li; IEEE Sensors Journal 2022 Vol. 22*

  ### 想解决的问题  
  1. 现有基于霍尔阵列的磁定位只能获得 5-D 位姿，无法感知绕胶囊磁化轴的自旋（roll）。  
  2. 仅靠 IMU 又无法估计绕重力轴的偏航（yaw），且纯积分会随时间漂移。  
  3. 临床需要在外部电磁驱动下，对无线磁控胶囊内镜（WMACE）实时、精准、无辐射地获取 6-D（3D 位置 + 3D 姿态）位姿，以实现闭环导航与诊疗。  

  ### 具体做法  
  #### 1. 系统硬件  
  1. WMACE 胶囊  
    * 直径 11 mm×长度 30 mm；环形永磁壳，磁矩 0.65 A·m²，与重力方向初始夹角设定 90°。  
    * 集成 6 轴 IMU（加速度计+陀螺仪）、摄像头、LED、BLE+AV 发射模块，实现视频与 IMU 数据无线传输。  
  2. 5 × 5 三轴霍尔传感器阵列  
    * 传感芯片：MLX90393（量程 5–50 mT）；阵列间距 d_H=8 mm。  
    * STM32 采集，最高 200 Hz 上传。  
  3. 外部电磁驱动：单线圈电磁铁（磁矩最高 289.7 A·m²）置于胶囊下方，用于运动控制并参与解耦计算。  

  #### 2. 5-D 位置 + 姿态估计（霍尔阵列）  
  1. 磁场模型：  
    B(r)=μ₀/4πr³[3(r·m)r/r²−m]；总场 B_s= B_e(电磁体) + B_c(胶囊)。  
  2. 两类优化算法：  
    a. 直接估计 Dir-Est：最小化 25 点×3 轴磁场测量与模型差异 c₁(rc,mc)（LMA 迭代）。  
    b. 二阶导 Sec-Der：对阵列磁场做五点二阶差分，构造 c₂(rc,mc)；对噪声更鲁棒。  
  3. 取 5×5·3-axis Dir-Est 为最优方案：MAE 0.22 mm（动态环轨实验）。  

  #### 3. 3-D 姿态估计（IMU）  
  1. 加速度计：解算 pitch_acc, roll_acc。  
  2. 磁矩方向：由上步霍尔估计的 m_c 求 yaw_mag, roll_mag。  
  3. 陀螺仪：积分得 ω_roll,pitch,yaw → roll_gyro 等。  
  4. 互补滤波 / 卡尔曼滤波融合  
    * 状态向量 x=[roll, bias_roll, pitch, bias_pitch, yaw, bias_yaw]ᵀ  
    * A,B,C 矩阵如公式(15)；过程噪声 Q、测量噪声 R 对角配置。  
    * 滤波方程(16)(17)输出 roll_kalm, pitch_kalm, yaw_kalm。  
  5. 关键假设：磁矩与重力初始垂直 ⇒ 霍尔提供 yaw，IMU 提供 roll，互补后获得全 3-D 姿态。  

  #### 4. 多传感器 6-D 融合流程  
  ```
  Hall阵列 → 5-D (rc,mc)                    ┐
                                            ├→ yaw_mag, roll_mag
  IMU → acc, gyro → pitch_acc,roll_acc,ω   ┘
  互补/卡尔曼滤波 → roll,pitch  (优先IMU)  
  LMA迭代 + yaw_mag 更新 → yaw  
  最终输出： rc(x,y,z) + 欧拉角(roll,pitch,yaw)
  ```

  ### 效果  
  1. **动态实验**：36 mm 高度下跟踪圆轨迹，位置 MAE 0.22 mm；三种 orientation 滤波后误差 0.3° 级。  
  2. **静态定位**（25–72 mm 高度）：  
    * 3-D 位置 MAE 1.463 ± 0.288 mm；72–152 mm 误差随距离增至 26 mm。  
  3. **静态姿态**：72 mm 高度，Kalman 滤波 roll 0.185°、pitch 0.169°、yaw 0.407° MAE。  
  4. **对比文献**：综合精度优于 2020-2022 年基于单磁阵/反向磁阵/传感手套等方法（表 III）。  
  5. **鲁棒性**：外部电磁场 0/57/290 A·m² 不影响定位精度；噪声 0.2 Gs 下模拟与实测一致。  

  ### 关键词  
  磁控胶囊内镜；6-D 位姿定位；霍尔阵列；IMU 融合；勒文伯格-马夸特；二阶磁场差分；互补滤波；卡尔曼滤波






































####
# About us - 关于我们
###### "己欲立而立人，己欲达而达人"
我们是一个由机器人与具身初学者组成的团队, 希望能够通过我们自己的学习、科研经验, 为志同道合的朋友提供一些帮助。欢迎更多朋友加入我们的项目, 也很欢迎交友、学术合作, 有任何问题, 可以联系邮箱 `yimouwu0@gmail.com` 或 `ymwu@surgery.cuhk.edu.hk`。

<p><b>🎠Contributors</b>: <a href="https://yimouwu.github.io/">吴贻谋 (25'港中文MPhil)</a> 
</p> 

<a href="https://github.com/yimouwu/Robotics-and-Embodied-AI-Review/contributors">
  <img src="https://contrib.rocks/image?repo=yimouwu/Robotics-and-Embodied-AI-Review" />
</a>

#####
> 关于本仓库的知识讨论，或者任何其他问题,欢迎联系吴贻谋（25'fall CUHK 医学院外科系 MPhil，方向为机器人（医疗手术）与具身智能）微信：yimouwu777 或 邮箱：yimouwu0@gmail.com

# 👍 Citation <a id="citation"></a>

如果本仓库对你的研究或学习有所帮助，请引用：
```bibtex
@misc{marvelousliteraturereview2025,
  title     = {Marvelous-Literature-Review},
  author    = {Yimou Wu},
  month     = {August},
  year      = {2025},
  url       = {https://github.com/yimouwu/Marvelous-Literature-Review},
}
```


# 🏷️ License <a id="license"></a>

This repository is released under the MIT license. See [LICENSE](./LICENSE) for additional details.



# ⭐️ Star History <a id="star-history"></a>

## Star History
[![Star History Chart](https://api.star-history.com/svg?repos=yimouwu/Marvelous-Literature-Review&type=Date)](https://star-history.com/#yimouwu/Marvelous-Literature-Review&Date)
