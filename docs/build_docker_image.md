# Build Docker Image

于 Repo 根目录下，在终端中分别使用：

```bash
docker build . --target rat-base -t rat-base:latest
```

```bash
docker build . --target rat-dev -t rat-dev:latest
```

构建基础容器、开发容器。

## 构建 `rat-dev-full:latest`

`latest-full` 需要对向架构的 `rat-base` 作为 sysroot 来源。

下面示例以在 `amd64` 主机上构建 `amd64 latest-full` 为例：

```bash
docker buildx build . --platform linux/amd64 --target rat-base -t rat-base:linux-amd64 --load
```

```bash
docker buildx build . --platform linux/arm64 --target rat-base -t rat-base:linux-arm64 --load
```

```bash
docker buildx build . --platform linux/amd64 --target rat-dev-full -t rat-dev-full:latest \
  --build-arg SYSROOT_IMAGE_AMD64=rat-base:linux-amd64 \
  --build-arg SYSROOT_IMAGE_ARM64=rat-base:linux-arm64 \
  --load
```

如果是在 `arm64` 主机上构建，则交换目标平台即可（`latest-full` 的 sysroot 方向相反）。

## 代理构建

部分国家和地区会阻断对 `github` 的连接，此时构建需要使用代理：

```bash
docker build . --target rat-dev -t rat-dev:latest \
  --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890
```

请自行把 `http://127.0.0.1:7890` 改为合适的代理地址。

如果不使用本机回环地址 (`127.0.0.1`) 作为代理地址，构建参数中的 `--network host` 可以省去。

## CI 自动构建（GitHub Actions）

`.github/workflows/docker-image.yml` 会在 GitHub 上构建并推送三个阶段的镜像：

| 镜像 | 阶段 | 内容 |
|---|---|---|
| `mide233/flyrat-base:latest` | `rat-base` | ROS 2 Jazzy 运行时依赖 |
| `mide233/flyrat-dev:latest` | `rat-dev` | 开发工具链（gcc-14 / clang-22 / cmake 4.2.3 / node 24） |
| `mide233/flyrat-dev-full:latest` | `rat-dev-full` | 交叉工具链 + 对向架构 sysroot + Gazebo Harmonic |

流程：`base`（两架构各自构建，按 digest 推送）→ `dev`（`rat-dev` + 用本次 `base` 的 digest 作为 sysroot 的 `rat-dev-full`）→ `promote`（把两架构 digest 合成多架构 manifest 并打 `latest`）。

镜像里没有单独的冒烟测试 job：`latest` 只在两个架构的 `rat-dev-full` 都构建成功后才会更新，构建失败不会覆盖已经在用的镜像；要验证工具链，拉下来跑一次 `.script/build_libflyrat.sh` / `.script/build_ros2.sh` 即可。

### 一次性配置

1. **Docker Hub 访问令牌**：Docker Hub → Account settings → Personal access tokens → Generate new token，权限选 *Read & Write*，复制 `dsec_...`。
2. **仓库 Secrets**：GitHub 仓库 → Settings → Secrets and variables → Actions → New repository secret：
   - `DOCKERHUB_USERNAME` = `mide233`
   - `DOCKERHUB_TOKEN` = 上一步的令牌
3. **（可选）arm64 runner**：workflow 用 `ubuntu-24.04-arm` 原生构建 arm64。公开仓库可直接使用；私有仓库需要账户支持 arm64 runner，否则把 matrix 里的 arm64 项删掉，或改用 `docker/setup-qemu-action` + `--platform linux/arm64`（会慢很多）。

不需要开启任何额外的"Actions 权限"开关：workflow 只用 `GITHUB_TOKEN` 之外的 Docker Hub secrets，`permissions` 已限制为 `contents: read`。

### 触发与查看

- 自动：推送到 `main` 且改动 `Dockerfile`、`.dockerignore`、workflow 本身或 `.script/template/env_setup.*` 时触发。
- 手动：仓库 → **Actions** → 左侧 **Build and push images** → **Run workflow**（选分支后运行）。
- 运行页里 `base (amd64)` / `dev (arm64)` 等 job 各自带缓存（`type=gha`），重复构建只重建变化的层；首次构建 `rat-dev-full` 的 sysroot 拷贝较慢，属正常。

### 使用构建好的镜像

```bash
docker pull mide233/flyrat-dev-full:latest
```

`docker-compose.yml` 里的服务名仍是 `image: rat-dev-full:latest`，两种用法任选：

```bash
# 1) 打本地标签，沿用现有 compose/devcontainer 配置
docker tag mide233/flyrat-dev-full:latest rat-dev-full:latest

# 2) 或直接改 compose 的 image 字段为 mide233/flyrat-dev-full:latest
```

拉下来的镜像里 `cmake` 已是 4.2.3（`/opt/cmake/bin` 在 `PATH` 前面），`gz` 仍需按项目约定用 `env -u GZ_CONFIG_PATH PATH="/usr/bin:$PATH" gz ...` 避开 ROS 的 vendor shim。
