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

- [ ] 阶段二：动作空间的跨越（Action Tokenization）
     - [ ] 理论
        - [ ] Action (动作): 了解 Imitation Learning (模仿学习) 和 Behavior Cloning (行为克隆) 的基本概念。


- [ ] 阶段三：具身数据与仿真环境（Sim-to-Real 桥梁）

- [ ] 阶段四：VLA 模型微调与端侧优化（Resource Optimization）