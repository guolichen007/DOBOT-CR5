# 代码传输与版本管理工作流

本文档说明 Windows 主机与 Ubuntu 笔记本之间的代码版本管理方案。

## 推荐网络拓扑

- Ubuntu 笔记本有线网口/USB 网卡：直连 CR5 控制柜网络，IP `192.168.110.100/24`，不设网关和 DNS
- Ubuntu 笔记本 Wi-Fi 或第二个网口：正常局域网/互联网访问，同时接收来自 Windows 主机的 SSH 连接
- Windows 主机通过 Ubuntu 笔记本的 Wi-Fi/LAN 地址与其通信
- 不要将 CR5 控制柜网络接口桥接到互联网接口

## 推荐版本管理流程

以 GitHub 作为权威远程仓库，同时可在 Ubuntu 笔记本上设置局域网 Git 远端作为快速推送通道。

### 在 Ubuntu 笔记本上设置局域网 Git 远端（可选）

```bash
sudo apt install -y openssh-server git
sudo systemctl enable --now ssh
mkdir -p ~/git/cr5_a4_ros1.git
git init --bare ~/git/cr5_a4_ros1.git
hostname -I
```

### 在 Windows 主机上添加局域网远端

```powershell
ssh-keygen -t ed25519
ssh <ubuntu用户名>@<笔记本局域网IP>
git remote add lab ssh://<ubuntu用户名>@<笔记本局域网IP>/home/<ubuntu用户名>/git/cr5_a4_ros1.git
git push -u lab main
git push -u origin main
```

### 在 Ubuntu 笔记本上克隆

```bash
git clone ~/git/cr5_a4_ros1.git ~/cr5_a4_ros1
cd ~/cr5_a4_ros1
./scripts/build.sh
```

### Windows 主机日常开发流程

```powershell
git switch -c feature/<功能名>
git add .
git commit -m "提交说明"
git push lab HEAD
git push origin HEAD
ssh <ubuntu用户名>@<笔记本局域网IP> "cd ~/cr5_a4_ros1 && git fetch && git switch feature/<功能名> && git pull --ff-only"
```

## 辅助工具

- 使用 VS Code Remote-SSH 连接 Ubuntu 笔记本，可在不手动复制文件的情况下编译、启动和查看日志
- 对于大型 rosbag 或模型文件，使用 `scp` 或 `rsync` 传输，不用于源代码同步

局域网远端配置完成后，可使用项目提供的 PowerShell 脚本快速部署：

```powershell
.\tools\windows\deploy_lab.ps1 `
  -LaptopHost <笔记本局域网IP> `
  -LaptopUser <ubuntu用户名>
```

该脚本将当前分支推送到笔记本的 Git 仓库并更新 `~/cr5_a4_ros1`，不执行编译或控制机械臂。
