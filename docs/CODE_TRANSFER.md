# Desktop-to-laptop development workflow

## Recommended topology

- Laptop Ethernet/USB-Ethernet: direct CR5 network, `192.168.110.100/24`, no gateway/DNS.
- Laptop Wi-Fi or second NIC: normal LAN/Internet and SSH access from the desktop.
- Desktop communicates with the laptop using the laptop Wi-Fi/LAN address.
- Do not bridge the CR5 interface to the Internet-facing interface.

## Recommended version flow

Use GitHub as the authoritative remote and add the Ubuntu laptop as a fast LAN Git remote.

On the Ubuntu laptop:

```bash
sudo apt install -y openssh-server git
sudo systemctl enable --now ssh
mkdir -p ~/git/cr5_a4_ros1.git
git init --bare ~/git/cr5_a4_ros1.git
hostname -I
```

On the Windows desktop repository:

```powershell
ssh-keygen -t ed25519
ssh <ubuntu-user>@<laptop-lan-ip>
git remote add lab ssh://<ubuntu-user>@<laptop-lan-ip>/home/<ubuntu-user>/git/cr5_a4_ros1.git
git push -u lab main
git push -u origin main
```

On the Ubuntu laptop:

```bash
git clone ~/git/cr5_a4_ros1.git ~/cr5_a4_ros1
cd ~/cr5_a4_ros1
./scripts/build.sh
```

Daily desktop flow:

```powershell
git switch -c feature/<name>
git add .
git commit -m "..."
git push lab HEAD
git push origin HEAD
ssh <ubuntu-user>@<laptop-lan-ip> "cd ~/cr5_a4_ros1 && git fetch && git switch feature/<name> && git pull --ff-only"
```

Use VS Code Remote-SSH to compile, launch and inspect logs on Ubuntu without copying files manually.
Use `scp` or `rsync` for large bags/models, not for source-code synchronization.

A convenience PowerShell script is included after the `lab` remote and laptop working clone are configured:

```powershell
.\tools\windows\deploy_lab.ps1 \
  -LaptopHost <laptop-lan-ip> \
  -LaptopUser <ubuntu-user>
```

It pushes the current branch to the laptop's bare Git repository and updates `~/cr5_a4_ros1` on the laptop. It does not compile or move the robot.
