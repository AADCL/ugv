# Scout Mini GitHub 上传配置与安全说明 V4.3

> 本文件只记录非敏感配置和标准流程。不得写入设备登录密码、私钥正文、GitHub Token、Cookie 或带认证信息的 URL。

## 1. 当前组织仓库

| 项目 | 当前值 |
|---|---|
| GitHub 网页 | `https://github.com/AADCL/ugv` |
| SSH 远程 | `git@github.com:AADCL/ugv.git` |
| 分支 | `main` |
| NX Git 克隆 | `/home/nrc19/github_upload/ugv_scout_wheeltech` |
| Scout 子目录 | `Scout_mini/` |
| Git 用户名 | `BAIOLED` |
| Git 邮箱 | `114551361+BAIOLED@users.noreply.github.com` |

组织策略已停用 Deploy Key，因此使用已添加到 AADCL 组织账户的专用 SSH 账户密钥。当前 NX 密钥路径为：

```text
/home/nrc19/.ssh/id_ed25519_github_aadcl_account
```

本文不保存公钥正文或指纹。密钥权限：

```bash
chmod 700 /home/nrc19/.ssh
chmod 600 /home/nrc19/.ssh/id_ed25519_github_aadcl_account
```

## 2. GitHub SSH 443

当前网络使用 `ssh.github.com:443`：

```bash
ssh -T -p 443 \
  -i /home/nrc19/.ssh/id_ed25519_github_aadcl_account \
  -o IdentitiesOnly=yes \
  -o Hostname=ssh.github.com \
  git@github.com
```

认证成功只表示账户密钥有效，仓库写权限仍由 AADCL 组织和仓库成员权限决定。

## 3. 提交前检查

```bash
cd /home/nrc19/github_upload/ugv_scout_wheeltech
git remote -v
git branch --show-current
git status --short
git diff --check
git diff --stat
```

只暂存本次 Scout 修改和必要的根 README：

```bash
git add Scout_mini README.md
git status --short
git diff --cached --check
git diff --cached --stat
```

提交前逐项确认没有地图、PCD、bag、日志、密码和密钥。

## 4. 提交与推送

```bash
git commit -m "Update Scout terrain mapping and navigation flow"

GIT_SSH_COMMAND="ssh -p 443 -i /home/nrc19/.ssh/id_ed25519_github_aadcl_account -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git push origin main
```

推送后验证：

```bash
git status --short
git rev-parse HEAD

GIT_SSH_COMMAND="ssh -p 443 -i /home/nrc19/.ssh/id_ed25519_github_aadcl_account -o IdentitiesOnly=yes -o Hostname=ssh.github.com" \
  git ls-remote origin refs/heads/main
```

本地 HEAD 与远端 `refs/heads/main` 应一致。

## 5. 禁止上传

- `/home/nrc19/.ssh/` 中的任何私钥；
- 密码、Token、Cookie、`.env` 和认证 URL；
- `build/`、`devel/`、`logs/`、`*.pyc`；
- `*.pcd`、`*.bag`、PGM/YAML 地图运行产物和传感器数据；
- Scout 车端个人配置或未经检查的第三方二进制；
- 与本次提交无关的 WheelTech 工作区变化。

发现密钥进入 Git 历史后，不能只删最新文件；必须立即撤销密钥并清理整个 Git 历史。
