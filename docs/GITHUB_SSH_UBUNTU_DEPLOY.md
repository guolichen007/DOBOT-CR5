# GitHub SSH 与 Ubuntu 部署指南

本文档适用于 Ubuntu 笔记本（Ubuntu 20.04 + ROS Noetic），通过 Wi-Fi 从 GitHub 拉取 DOBOT-CR5 代码，有线网口直连 DOBOT CR5 控制柜。

---

## 1. 网络结构

```text
Ubuntu 笔记本
├── Wi-Fi ────────── 互联网 / GitHub
└── 有线网口 ─────── 192.168.110.100/24 ── 控制柜 192.168.110.214
      (不设网关、不设 DNS，默认路由走 Wi-Fi)
```

检查当前网络：

```bash
ip -br addr
ip route
```

验证连通性：

```bash
ping -c 3 github.com        # 应通过 Wi-Fi 成功
ping -c 3 192.168.110.214    # 应通过有线成功
```

正确路由示例：

```text
default via 192.168.1.1 dev wlp2s0          ← Wi-Fi 网关
192.168.110.0/24 dev enp3s0 src 192.168.110.100   ← 有线，无网关
```

如果默认路由指向有线网口，说明有线配置了网关，需要移除后重试。

---

## 2. 安装 Git 和 SSH

```bash
sudo apt update
sudo apt install -y git openssh-client

git --version
ssh -V

git config --global init.defaultBranch main
git config --global core.autocrlf input
```

> 注意：不要使用 `sudo git clone`、`sudo git pull` 或 `sudo git commit`。

---

## 3. 创建专用 SSH 密钥

密钥路径：`~/.ssh/id_ed25519_github_dobot_cr5_ubuntu`

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh

if [ ! -f ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu ]; then
  ssh-keygen -t ed25519 \
    -C "guolichen007@gmail.com Ubuntu DOBOT-CR5" \
    -f ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
fi

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
```

---

## 4. 配置 SSH Host 别名

以下命令会检查 `~/.ssh/config` 中是否已存在 `github-dobot-cr5`，如不存在则追加：

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

设置权限：

```bash
chmod 600 ~/.ssh/config
chmod 600 ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu
chmod 644 ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu.pub
```

---

## 5. 添加公钥到 GitHub

显示公钥并复制：

```bash
cat ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu.pub
```

将输出的完整内容添加到 GitHub：

1. 打开 <https://github.com/settings/keys>
2. 点击 **New SSH key**
3. Title 填写：`Ubuntu-Laptop-DOBOT-CR5`
4. Key type 选择：**Authentication Key**
5. 粘贴公钥内容，点击 **Add SSH key**

---

## 6. 测试 SSH 连接

```bash
ssh -T git@github-dobot-cr5
```

首次连接会提示确认主机指纹，输入 `yes`。

成功输出应包含：

```text
Hi guolichen007! You've successfully authenticated, but GitHub does not provide shell access.
```

> GitHub 不提供 shell，命令返回非零退出码是正常的，以认证文本为准。

---

## 7. 首次克隆

```bash
cd ~
git clone git@github-dobot-cr5:guolichen007/DOBOT-CR5.git cr5_ros1_ws
cd ~/cr5_ros1_ws

git config pull.ff only
git remote -v
git branch -vv
git status
```

如果 `~/cr5_ros1_ws` 已存在但不是本仓库，先备份：

```bash
mv ~/cr5_ros1_ws ~/cr5_ros1_ws_backup_$(date +%Y%m%d_%H%M%S)
```

---

## 8. 首次编译

```bash
cd ~/cr5_ros1_ws
chmod +x scripts/*.sh
mkdir -p ~/cr5_test_logs

./scripts/build.sh 2>&1 | \
  tee ~/cr5_test_logs/build_first_$(date +%Y%m%d_%H%M%S).log
```

编译后检查：

```bash
git status --short
```

正常情况下不应出现 `build/`、`devel/` 等生成目录（已被 `.gitignore` 忽略）。

---

## 9. 机械臂网络配置

先查看有线接口名：

```bash
ip -br link
```

然后使用项目脚本配置有线网口（将 `enp3s0` 替换为实际接口名）：

```bash
cd ~/cr5_ros1_ws
sudo ./scripts/configure_robot_network.sh enp3s0
./scripts/network_check.sh
```

> **重要提醒：**
> - 有线网口只连接控制柜，不连接其他网络设备
> - 有线网口不配置默认网关
> - GitHub 网络流量应走 Wi-Fi
> - 不要将控制柜网络桥接到互联网

---

## 10. 日常拉取与编译

```bash
cd ~/cr5_ros1_ws
git status --short
git pull --ff-only origin main
./scripts/build.sh
```

如果主机推送了功能分支：

```bash
cd ~/cr5_ros1_ws
git fetch --prune origin
git branch -a
git switch <分支名>
git pull --ff-only
./scripts/build.sh
```

---

## 11. 常见问题

### 11.1 Permission denied (publickey)

```bash
# 检查密钥是否已加入 agent
ssh-add -l

# 如果列表中没有，手动添加
ssh-add ~/.ssh/id_ed25519_github_dobot_cr5_ubuntu

# 检查 ~/.ssh/config 中 Host 别名是否正确
cat ~/.ssh/config

# 确认 GitHub 上已添加对应公钥
```

### 11.2 Host key verification failed

```bash
# 删除旧指纹后重新连接
ssh-keygen -R github.com
ssh -T git@github-dobot-cr5
# 首次提示时确认指纹与 github.com 一致后输入 yes
```

### 11.3 Repository not found

- 检查 GitHub 账号 `guolichen007` 是否有仓库访问权限
- 确认 SSH 登录的是正确的账号：`ssh -T git@github-dobot-cr5`
- 确认仓库地址为 `git@github-dobot-cr5:guolichen007/DOBOT-CR5.git`

### 11.4 Your local changes would be overwritten

```bash
# 查看本地修改
git status

# 方案一：提交本地修改
git add -A
git commit -m "保存本地修改"

# 方案二：暂存后拉取
git stash
git pull --ff-only origin main
git stash pop
```

> 不要使用 `git push --force` 或 `git reset --hard`，除非你完全理解后果。

### 11.5 脚本出现 ^M（Windows 换行符）

```bash
# 在主机（Windows）端修复后推送，或本地临时处理
sed -i 's/\r$//' scripts/*.sh
```

根本解决方案：确认 `.gitattributes` 已包含 `*.sh text eol=lf`，然后在主机执行：

```bash
git add --renormalize .
git commit -m "修复换行符"
git push
```

### 11.6 GitHub 能访问但控制柜不通

```bash
# 检查有线 IP
ip -br addr show dev enp3s0

# 检查网线连接
ip link show enp3s0

# 直接 ping 控制柜
ping -c 3 192.168.110.214
```

如果 ping 不通，检查网线是否插好、有线接口是否配置了 `192.168.110.100/24`。

### 11.7 控制柜能通但 GitHub 不通

```bash
# 检查默认路由
ip route | head -1
```

如果默认路由指向有线网口（如 `default via 192.168.110.xxx dev enp3s0`），说明有线配置了网关。移除后默认路由应自动回到 Wi-Fi：

```bash
sudo ip route del default dev enp3s0
```

永久修复：编辑 Netplan 配置，确保有线接口不设 `routes` 或 `gateway4`。
