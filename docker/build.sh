#!/usr/bin/env bash

set -euo pipefail

# 建置並啟動（detach）
# 用法：
#   ./docker/build.sh            # 依照 docker/docker-compose.yml 建置，然後 up -d
#   ./docker/build.sh --no-cache # 傳遞給 build 的參數（不會傳給 up）

export DOCKER_BUILDKIT=1

docker compose -f docker/docker-compose.yml build "$@"
docker compose -f docker/docker-compose.yml up -d
echo "Build 完成並已以背景模式啟動。查看狀態：docker compose -f docker/docker-compose.yml ps"
