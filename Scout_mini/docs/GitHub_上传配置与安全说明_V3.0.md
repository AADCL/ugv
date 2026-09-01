# Scout Mini GitHub上传配置与安全说明

> 本文件记录上传代码所需的非敏感配置和标准流程。严禁把私钥正文、设备登录密码、GitHub Token、地图、rosbag或运行日志提交到仓库。

## 1. 当前仓库信息

| 项目 | 当前值 |
|---|---|
| GitHub网页 | `https://github.com/BAIOLED/AADCL_UAV_UGV.git` |
| SSH远程地址 | `git@github.com:BAIOLED/AADCL_UAV_UGV.git` |
| 远程名 | `origin` |
| 主分支 | `main` |
| 车端Git克隆 | `~/github_upload/AADCL_UAV_UGV` |
| 车端ROS工作空间 | `~/livox_fastlio` |
| Git用户名 | `BAIOLED` |
| Git邮箱 | `114551361+BAIOLED@users.noreply.github.com` |
| 核对时HEAD | `3d8229668ebd3950cc70668f3fb4547967659ef1` |

## 2. SSH密钥信息

| 项目 | 当前值 |
|---|---|
| 私钥路径 | `~/.ssh/id_ed25519_github_aadcl` |
| 公钥路径 | `~/.ssh/id_ed25519_github_aadcl.pub` |
| 算法 | ED25519 |
| 指纹 | `SHA256:8ZoRZ9YsPS15FfZyp7atPzcYtfj8JKL5eZ+1i+4AlSo` |
| 注释 | `nvidia-scout-mini-aadcl` |

GitHub账户中需要添加的公钥是：

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIPdHBRJz2YnBHi4IDsUjpiJniFBPFoWMmxe/aq1MckY/ nvidia-scout-mini-aadcl
```

私钥权限必须为600，公钥可以为644：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/id_ed25519_github_aadcl
chmod 644 ~/.ssh/id_ed25519_github_aadcl.pub
```

私钥只保存在受控设备或离线加密备份中，不能通过聊天、邮件或Git传输。设备SSH登录密码与GitHub认证无关，也不能写入仓库。

## 3. 为什么使用GitHub SSH 443端口

当前网络可能阻止标准SSH 22端口，因此使用`ssh.github.com:443`。测试认证：

```bash
ssh -T -p 443 \
  -i ~/.ssh/id_ed25519_github_aadcl \
  -o IdentitiesOnly=yes \
  -o Hostname=ssh.github.com \
  git@github.com
```

首次连接确认GitHub主机指纹后，成功时会显示认证成功但不提供shell。

## 4. 每次上传的完整流程

先把需要提交的工作空间文件同步到Git克隆。不要复制`build/`、`devel/`、地图、PCD、bag和日志。

```bash
cd ~/github_upload/AADCL_UAV_UGV

git status --short
git diff --check
git diff --stat

git add Scout_mini
git status --short
git diff --cached --check
git diff --cached --stat

git commit -m "说明本次修改内容"

GIT_SSH_COMMAND="ssh -p 443 -i $HOME/.ssh/id_ed25519_github_aadcl -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git push origin main
```

不要直接使用`git add -A`提交未经检查的整个用户目录。提交前必须查看暂存文件，确认不含秘密和生成文件。

## 5. 推荐的SSH配置

可在`~/.ssh/config`加入：

```sshconfig
Host github.com
  HostName ssh.github.com
  Port 443
  User git
  IdentityFile ~/.ssh/id_ed25519_github_aadcl
  IdentitiesOnly yes
```

设置权限：

```bash
chmod 600 ~/.ssh/config
```

此后可直接运行：

```bash
cd ~/github_upload/AADCL_UAV_UGV
git push origin main
```

如果不希望改变其他GitHub仓库的SSH行为，继续使用第4节的单次`GIT_SSH_COMMAND`，不要写全局配置。

## 6. 新设备恢复仓库

先安全地把专用私钥放入新设备的`~/.ssh/id_ed25519_github_aadcl`并设置600权限，然后：

```bash
mkdir -p ~/github_upload
cd ~/github_upload

GIT_SSH_COMMAND="ssh -p 443 -i $HOME/.ssh/id_ed25519_github_aadcl -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git clone git@github.com:BAIOLED/AADCL_UAV_UGV.git

cd AADCL_UAV_UGV
git config user.name "BAIOLED"
git config user.email "114551361+BAIOLED@users.noreply.github.com"
```

若私钥丢失，不要从公开仓库恢复。应在GitHub中删除旧Deploy Key/SSH Key，重新生成密钥对并添加新公钥。

## 7. 上传后验证

```bash
cd ~/github_upload/AADCL_UAV_UGV
git status --short
git rev-parse HEAD

GIT_SSH_COMMAND="ssh -p 443 -i $HOME/.ssh/id_ed25519_github_aadcl -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git ls-remote origin refs/heads/main
```

`git status --short`应为空，本地HEAD与远端`refs/heads/main`哈希应一致。

## 8. 禁止上传内容

- `~/.ssh/`中的任何私钥；
- 密码、Token、Cookie、`.env`和带认证信息的URL；
- `build/`、`devel/`、`logs/`、`*.pyc`；
- `*.pcd`、`*.bag`、地图运行产物和大体积传感器数据；
- 与Scout Mini无关的WheelTech源码或文档；
- 未核对来源的第三方二进制和个人配置。

发现秘密被提交后，不能只删除最新文件；必须立即撤销对应密钥，并按Git历史清理流程处理。
