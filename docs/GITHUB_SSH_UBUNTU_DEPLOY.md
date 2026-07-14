# GitHub SSH 与 Ubuntu 部署指南

本文档指导你在 Ubuntu 笔记本上完成 DOBOT-CR5 项目的全部配置，包括 SSH 密钥、GitHub 连接、双网卡设置、仓库克隆、首次编译和机械臂网络配置。

适用环境：Ubuntu 20.04 LTS + ROS Noetic。

---

## 目录

1. 网络结构
2. 安装 Git 和 SSH
3. 创建专用 SSH 密钥
4. 配置 SSH Host 别名
5. 添加公钥到 GitHub
6. 测试 SSH 连接
7. 首次克隆
8. 首次编译
9. 机械臂网络配置
10. 日常拉取与编译
11. 功能分支工作流
12. 常见问题

---

## 1. 网络结构

Ubuntu 笔记本采用双网卡设计：

```text
Ubuntu 笔记本
├── Wi-Fi ────────────── 互联网 / GitHub（默认路由）
└── 有线网口 ──────────── 192.168.110.100/24 ── 控制柜 192.168.110.214
                          （不设网关、不设 DNS）
```

有线网口仅用于与 DOBOT CR5 控制柜通信，所有互联网流量（包括 GitHub）通过 Wi-Fi 走默认路由。

### 1.1 检查当前网络状态

```bash
ip -br addr
ip route
```

### 1.2 验证双网卡连通性

```bash
ping -c 3 github.com        # 应通过 Wi-Fi 成功
ping -c 3 192.168.110.214    # 应通过有线成功
```

### 1.3 正确的路由表示例

```text
default via 192.168.1.1 dev wlp2s0                ← Wi-Fi 网关，负责互联网流量
192.168.110.0/24 dev enp3s0 src 192.168.110.100    ← 有线，仅用于控制柜通信
```

如果默认路由指向有线网口（如 `default via 192.168.110.xxx dev enp3s0`），说明有线接口配置了网关，需要移除后重试。参见第 12.7 节。

---

## 2. 安装 Git 和 SSH

```bash
sudo apt update
sudo apt install -y git openssh-client

# 验证安装
git --version
ssh -V

# 配置 Git
git config --global init.defaultBranch main
git config --global core.autocrlf input
```

> 注意：本项目的所有 Git 操作（clone、pull、commit 等）都不要使用 `sudo`。使用 `sudo` 会导致文件权限问题，后续操作可能需要一直使用 `sudo`。

---

## 3. 创建专用 SSH 密钥

为本项目创建独立的 SSH 密钥，不与其他用途混合：

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
```

### 3.1 生成密钥

密钥路径：`~/.ssh/id_ed25519_github_dobot_cr5_ubuntu`

```bash
if [ ! -f ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu ]; then
  ssh-keygen -t ed25519 \
    -C "guolichen007@gmail.com Ubuntu DOBOT-CR5" \
    -f ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
fi
```

提示 `Enter passphrase` 时可以直接按 Enter 跳过（不设密码），也可以输入一个密码保护私钥。如果设置了密码，每次开机后首次使用 SSH 时需要输入。

### 3.2 启动 ssh-agent 并加载密钥

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
```

如果设置了 passphrase，此时需要输入。

---

## 4. 配置 SSH Host 别名

使用独立的 Host 别名 `github-dobot-cr5` 指向 `github.com`，确保 Git 操作使用本项目的专用密钥。

### 4.1 追加配置（不覆盖现有文件）

```bash
CONFIG="$HOME/.ssh/config"
if [ ! -f "$CONFIG" ]; then
  touch "$CONFIG"
fi

if ! grep -q "^Host github-dobot-cr5$" "$CONFIG" 2>/dev/null; then
  cat >> "$CONFIG" << 'SSHEOF'

Host github-dobot-cr5
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
    IdentitiesOnly yes
    AddKeysToAgent yes
SSHEOF
fi
```

> 此命令只会追加，不会覆盖 `~/.ssh/config` 中已有的其他 Host 配置。

### 4.2 设置文件权限

SSH 对文件权限要求严格，权限过宽会导致 SSH 拒绝使用密钥：

```bash
chmod 600 ~/.ssh/config
chmod 600 ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
chmod 644 ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu.pub
```

---

## 5. 添加公钥到 GitHub

### 5.1 显示公钥

```bash
cat ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu.pub
```

复制输出的完整内容（以 `ssh-ed25519` 开头的整行）。

### 5.2 在 GitHub 上添加

1. 浏览器打开 <https://github.com/settings/keys>
2. 点击 **New SSH key**
3. Title 填写：`Ubuntu-Laptop-DOBOT-CR5`（方便日后识别）
4. Key type 选择：**Authentication Key**
5. 将公钥内容粘贴到 Key 文本框
6. 点击 **Add SSH key**

> 只添加 `.pub` 公钥文件的内容，绝不添加没有扩展名的私钥文件。

---

## 6. 测试 SSH 连接

```bash
ssh -T git@github-dobot-cr5
```

首次连接会提示确认 GitHub 主机指纹，输入 `yes`。

成功输出应包含：

```text
Hi guolichen007! You've successfully authenticated, but GitHub does not provide shell access.
```

> GitHub 不提供 shell 访问，因此 SSH 命令可能返回非零退出码，这是正常的。判断是否成功的依据是输出中包含认证成功信息。

---

## 7. 首次克隆

### 7.1 如果 ~/cr5_ros1_ws 目录不存在

```bash
cd ~
git clone git@github-dobot-cr5:guolichen007/DOBOT-CR5.git cr5_ros1_ws
cd ~/cr5_ros1_ws
```

### 7.2 如果 ~/cr5_ros1_ws 已存在但不是本仓库

先备份：

```bash
mv ~/cr5_ros1_ws ~/cr5_ros1_ws_backup_$(date +%Y%m%d_%H%M%S)
```

然后重新执行第 7.1 节的克隆命令。

### 7.3 克隆后配置

```bash
cd ~/cr5_ros1_ws
git config pull.ff only
```

此配置确保 `git pull` 在无法快进时拒绝合并，避免意外产生合并提交。

### 7.4 验证

```bash
git remote -v
# 应显示：
# origin  git@github-dobot-cr5:guolichen007/DOBOT-CR5.git (fetch)
# origin  git@github-dobot-cr5:guolichen007/DOBOT-CR5.git (push)

git branch -vv
# 应显示 main 分支跟踪 origin/main

git status
# 应显示 working tree clean
```

---

## 8. 首次编译

```bash
cd ~/cr5_ros1_ws

# 确保脚本有执行权限
chmod +x scripts/*.sh

# 创建日志目录
mkdir -p ~/cr5_test_logs

# 执行编译并记录日志
./scripts/build.sh 2>&1 | \
  tee ~/cr5_test_logs/build_first_$(date +%Y%m%d_%H%M%S).log
```

编译完成后检查：

```bash
git status --short
```

正常情况下不应出现 `build/`、`devel/` 等目录（已被 `.gitignore` 忽略）。如果出现，说明 `.gitignore` 可能未生效，需检查。

### 8.1 加载工作空间环境

编译成功后，加载 ROS 环境：

```bash
source devel/setup.bash
```

建议将此行添加到 `~/.bashrc` 中，每次打开终端自动加载：

```bash
echo "source ~/cr5_ros1_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 9. 机械臂网络配置

### 9.1 查看有线接口名

```bash
ip -br link
```

找到连接控制柜的有线接口（通常为 `enp3s0`、`eth0` 或类似名称）。

### 9.2 使用项目脚本配置

将 `enp3s0` 替换为实际的有线接口名