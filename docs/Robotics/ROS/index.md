# ROS

> 机器人操作系统学习笔记，涵盖 ROS1 / ROS2 及 Linux 基础。

---

## ROS 概览

| 特性 | ROS1 (Noetic) | ROS2 (Humble / Jazzy) |
| :--- | :--- | :--- |
| 通信中间件 | 自研 TCPROS / UDPROS | DDS（Data Distribution Service） |
| 多机器人 | 需手动配置 | 原生支持 |
| 实时性 | 不支持 | 支持（配合 RTOS） |
| 生命周期节点 | 无 | 有 |
| Python 版本 | Python 2/3 | Python 3 |
| 构建工具 | `catkin` | `colcon` |
| 推荐 Ubuntu | 20.04 | 22.04 / 24.04 |

---

## Linux 常用命令速查

### 文件与目录

| 命令 | 说明 | 示例 |
| :--- | :--- | :--- |
| `ls -la` | 列出所有文件（含隐藏） | `ls -la ~/catkin_ws/src/` |
| `cd` | 切换目录 | `cd ~/catkin_ws` |
| `pwd` | 显示当前路径 | |
| `mkdir -p` | 递归创建目录 | `mkdir -p src/my_pkg/launch` |
| `cp -r` | 递归复制 | `cp -r pkg_a/ pkg_b/` |
| `mv` | 移动 / 重命名 | `mv old.py new.py` |
| `rm -rf` | 强制递归删除 | `rm -rf build/ devel/` |
| `find` | 查找文件 | `find . -name "*.launch"` |
| `tree -L 2` | 树形展示目录（2层） | `tree -L 2 src/` |

### 文本处理

| 命令 | 说明 | 示例 |
| :--- | :--- | :--- |
| `cat` | 查看文件内容 | `cat CMakeLists.txt` |
| `head -n 20` | 查看前 20 行 | `head -n 20 log.txt` |
| `tail -f` | 实时追踪日志 | `tail -f ~/.ros/log/latest/rosout.log` |
| `grep -rn` | 递归搜索关键词 | `grep -rn "subscriber" src/` |
| `nano` / `vim` | 终端编辑器 | `nano config.yaml` |

### 权限与用户

| 命令 | 说明 | 示例 |
| :--- | :--- | :--- |
| `chmod +x` | 添加可执行权限 | `chmod +x scripts/run.py` |
| `chown` | 修改所有者 | `chown -R $USER:$USER ~/ws/` |
| `sudo` | 以管理员身份执行 | `sudo apt install ros-humble-...` |

### 系统与进程

| 命令 | 说明 | 示例 |
| :--- | :--- | :--- |
| `ps aux \| grep ros` | 查找 ROS 进程 | |
| `kill -9 <PID>` | 强制终止进程 | |
| `htop` | 交互式进程监控 | |
| `df -h` | 查看磁盘空间 | |
| `free -h` | 查看内存使用 | |
| `uname -a` | 查看系统信息 | |
| `lsb_release -a` | 查看 Ubuntu 版本 | |

### 网络（多机通信常用）

| 命令 | 说明 | 示例 |
| :--- | :--- | :--- |
| `ifconfig` / `ip a` | 查看 IP 地址 | |
| `ping` | 测试网络连通性 | `ping 192.168.1.100` |
| `ssh` | 远程登录 | `ssh user@192.168.1.100` |
| `scp` | 远程拷贝文件 | `scp file.bag user@host:~/` |
| `export ROS_MASTER_URI` | 设置 ROS1 主节点地址 | `export ROS_MASTER_URI=http://192.168.1.100:11311` |
| `export ROS_IP` | 设置本机 IP | `export ROS_IP=192.168.1.50` |

### 包管理

```bash
sudo apt update                          # 更新软件源
sudo apt install <package>               # 安装软件包
sudo apt remove <package>                # 卸载
dpkg -l | grep ros                       # 查看已安装的 ROS 包
pip install <package>                    # Python 包安装
```

### Shell 环境

```bash
source ~/.bashrc                         # 重新加载配置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc   # 写入环境变量
alias cb='cd ~/colcon_ws && colcon build' # 自定义快捷命令
export | grep ROS                        # 查看 ROS 相关环境变量
```