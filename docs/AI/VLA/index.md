# VLA

## VLA 的核心运行机制
VLA 模型的本质是将机器人的连续控制问题，转化为类似大语言模型（LLM）的“下一个词预测”（Next-token prediction）问题。

输入端（Vision & Language）： 接收来自摄像头的环境图像/视频流，以及人类的自然语言指令（如：“把红色的苹果放到抽屉里”）。图像通常经过视觉编码器（如 ViT 或 SigLIP）处理成视觉 Token，与文本 Token 拼接。

处理中枢（LLM Backbone）： 多模态大模型对这些 Token 进行自回归推理，理解场景语义和物理关系。

输出端（Action Tokenization）： 模型不输出文本，而是输出动作 Token。研究人员会将机器人底层的连续控制信号（例如末端执行器的 XYZ 坐标、姿态、夹爪开合度、关节电机角度等）离散化（通常称为 Action Quantization 或 Tokenization），映射到大模型的词表中。

## 学习路线
- [ ] 阶段一：打通多模态感知（VLM 的部署与视觉理解）
    - [ ] 理论
        - [ ] Vision (视觉): 了解 ViT (Vision Transformer) 和 CLIP。VLA 极其依赖 CLIP 将图像和文本对齐。
        - [ ] Language (语言): 理解 Transformer 的 Decoder 架构 (GPT) 和 Tokenization (分词) 机制。
        - [ ] 阅读 LLaVA 系列论文（LLaVA-1.5 或 LLaVA-NeXT），理解视觉编码器（如 CLIP/SigLIP）通过投影层（MLP）接入 LLM 的架构。
        - [ ] 复习计算机视觉中的图像分割与目标识别算法，理解空间几何与语义信息的提取方式。
    - [ ] 实践
        - [ ] 本地或服务器部署一个轻量级的开源 VLM（例如 Qwen-VL-Chat 或 LLaVA）。输入包含复杂场景的图像，测试其对特定物体位置、空间关系的理解能力（例如：“图中红色的方块在绿色圆球的什么相对位置？”）。
        - [ ] 暂不看完整的 OpenVLA。去 GitHub 看 OpenVLA 的前身框架 Prismatic VLMs，或者 Hugging Face 上的 SmolVLM。下载它们的视觉编码器部分（如 SigLIP + DINOv2 的融合代码），弄清楚图像是怎么被切成 Patch 并转变成 LLM 能看懂的向量的。


- [ ] 阶段二：动作空间的跨越（Action Tokenization）
     - [ ] 理论
        - [ ] Action (动作): 了解 Imitation Learning (模仿学习) 和 Behavior Cloning (行为克隆) 的基本概念。
        - [ ] 精读 RT-1 (Robotics Transformer 1) 论文的核心章节，重点看它如何将机器人的连续动作（如 xyz 坐标、末端姿态）转换为离散的 Token。
        - [ ] 学习“动作分词”（Action Tokenization）中的 Binning 策略（例如将 -1.0 到 1.0 的连续控制区间均匀划分为 256 个独立的 Token）。
        - [ ]学习扩散策略（Diffusion Policy）。
    - [ ] 实践
        - [ ] 打开 OpenVLA 的 GitHub 源码，直接搜索并阅读 action_tokenizer.py 或类似的数据处理脚本，看它怎么把浮点数映射成 256 个特殊 Token（Binning 策略）。
        - [ ] 对比阅读 Octo 的架构，看看扩散模型是怎么处理连续动作流的。


- [ ] 阶段三：具身数据与仿真环境（Sim-to-Real 桥梁）
    - [ ] 理论
        - [ ] 了解 RLDS (TFDS) 数据集格式，这是 Google 系和 Open X-Embodiment 数据集的主流格式。
        - [ ] 学习 Sim-to-Real（仿真到现实）的跨越难点，特别是动力学参数的随机化（Domain Randomization）。
    - [ ] 实践
        - [ ] 下载并解析一小部分 Open X-Embodiment (OXE) 数据集，提取其中的图像、文本指令和动作轨迹数据，并进行可视化。
        - [ ] 搭建一个基础的物理仿真环境（推荐 Isaac Sim 或 MuJoCo），在环境中导入一个简单的机械臂或移动底盘模型。


- [ ] 阶段四：VLA 模型微调与端侧优化（Resource Optimization）
    - [ ] 理论
        - [ ] 精读 OpenVLA 论文，学习其基于 Llama-2/3 架构的训练策略。
        - [x] 矩阵低秩分解（如 LoRA 的数学原理）以及低比特量化（如 INT8/INT4 量化的误差分析与校准）。
    - [ ] 实践
        - [ ] 使用 Hugging Face 的 PEFT 库，结合阶段三提取的少量数据集，通过 LoRA 对 OpenVLA 进行微调（只训练投影层和部分注意力权重）。
        - [ ] 将你微调好的轻量级模型接入 Isaac Sim 或其他开源仿真器，测试它能否在虚拟环境里成功抓起一个杯子。



## 开源VLA模型

### 1、OpenVLA

特点： 目前最具代表性、也是被工业界和学术界引用最多的 7B（70亿参数）端到端自回归 VLA 模型。基于 Llama 架构，使用了海量的 Open X-Embodiment (OXE) 数据集训练。

学习价值： 它是学习“如何将动作离散化为 Token”的教科书。它的代码库对微调（特别是 LoRA 等低秩微调）的支持非常友好。

### 2、Octo 

特点： 与 OpenVLA 输出离散 Token 不同，Octo 采用了扩散策略（Diffusion Policy）。它直接输出连续的动作轨迹，动作更加平滑。此外，它极其轻量（有几十兆参数的版本）。

学习价值： 如果你想了解非自回归（Non-autoregressive）的 VLA 是怎么运作的，以及如何解决机器人动作卡顿的问题，Octo 是首选。

### 3、SmolVLA / TinyVLA 

特点： 由 Hugging Face 等社区推动的轻量级 VLA 模型（通常在 500M 参数以下），专为平民玩家和消费级硬件设计，可以直接在笔记本电脑上跑。

学习价值： 非常适合作为你在计算资源有限时的“跑通全流程”的练手项目。