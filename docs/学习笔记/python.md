# :material-language-python: Python 学习记录

!!! tip "关于这篇笔记"
    这是一份 Python 学习的活文档，持续更新中。
    涵盖基础语法、进阶技巧和常用库的使用心得。

---

## 基础语法

### 变量与数据类型

Python 是**动态类型**语言，变量不需要声明类型：

```python title="基础类型示例" linenums="1"
name: str = "nian"       # 字符串
age: int = 24            # 整数
gpa: float = 3.85        # 浮点数
is_grad: bool = True     # 布尔值
skills: list = ["Python", "ROS2", "PyTorch"]  # 列表
```

!!! note "类型注解"
    Python 3.5+ 支持类型注解（Type Hints），虽然不强制，
    但在大型项目中**强烈推荐**使用，配合 `mypy` 可以做静态类型检查。

---

### 条件与循环

=== "if-elif-else"

    ```python
    score = 92
    if score >= 90:
        grade = "A"
    elif score >= 80:
        grade = "B"
    else:
        grade = "C"
    print(f"成绩等级: {grade}")
    ```

=== "for 循环"

    ```python
    fruits = ["apple", "banana", "cherry"]
    for i, fruit in enumerate(fruits):
        print(f"{i}: {fruit}")
    ```

=== "列表推导式"

    ```python
    # 一行代码生成平方数列表
    squares = [x**2 for x in range(10) if x % 2 == 0]
    # 结果: [0, 4, 16, 36, 64]
    ```

---

## 进阶技巧

### 装饰器（Decorator）

!!! example "计时装饰器"

    ```python title="timer.py" linenums="1"
    import time
    from functools import wraps

    def timer(func):
        """测量函数执行时间的装饰器"""
        @wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            elapsed = time.perf_counter() - start
            print(f"⏱ {func.__name__} 耗时 {elapsed:.4f}s")
            return result
        return wrapper

    @timer
    def train_model():
        time.sleep(1.5)  # 模拟训练
        return "训练完成"
    ```

???+ info "常见内置装饰器"
    | 装饰器 | 用途 |
    | :--- | :--- |
    | `@staticmethod` | 静态方法，不需要 `self` |
    | `@classmethod` | 类方法，第一个参数是 `cls` |
    | `@property` | 将方法变成属性访问 |
    | `@functools.lru_cache` | 函数结果缓存 |

---

### 常用数据科学库

=== ":material-numpy: NumPy"

    ```python
    import numpy as np
    
    # 创建数组并计算
    a = np.array([1, 2, 3, 4, 5])
    print(f"均值: {a.mean()}, 标准差: {a.std():.2f}")
    
    # 矩阵运算
    A = np.random.randn(3, 3)
    eigenvalues = np.linalg.eigvals(A)
    ```

=== ":material-chart-line: Matplotlib"

    ```python
    import matplotlib.pyplot as plt
    import numpy as np
    
    x = np.linspace(0, 2 * np.pi, 100)
    plt.plot(x, np.sin(x), label="sin(x)")
    plt.plot(x, np.cos(x), label="cos(x)")
    plt.legend()
    plt.title("三角函数")
    plt.savefig("trig.png", dpi=150)
    ```

=== ":material-fire: PyTorch"

    ```python
    import torch
    import torch.nn as nn
    
    # 简单全连接网络
    model = nn.Sequential(
        nn.Linear(784, 256),
        nn.ReLU(),
        nn.Dropout(0.2),
        nn.Linear(256, 10),
    )
    print(f"参数量: {sum(p.numel() for p in model.parameters()):,}")
    ```

---

## 实用代码片段

!!! warning "注意 Python 版本"
    以下代码示例基于 **Python 3.10+**，部分语法（如 `match-case`）
    在旧版本中不可用。

### 文件读写（推荐方式）

```python title="安全读写文件" linenums="1"
from pathlib import Path

# ✅ 推荐：使用 pathlib
config_path = Path("config") / "settings.yaml"
content = config_path.read_text(encoding="utf-8")

# 写入文件
output = Path("output/result.txt")
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text("Hello, World!", encoding="utf-8")
```

---

## 待学习清单

- [x] 基础语法与数据类型
- [x] 函数与装饰器
- [ ] 异步编程 `asyncio`
- [ ] 元编程与描述符
- [ ] CPython 源码阅读

---

## 数学公式示例

梯度下降更新规则：

$$
\theta_{t+1} = \theta_t - \eta \nabla_\theta \mathcal{L}(\theta_t)
$$

其中 $\eta$ 为学习率，$\mathcal{L}$ 为损失函数。

---

!!! quote "写在最后"
    > "Talk is cheap. Show me the code." — Linus Torvalds