#!/usr/bin/env bash
# Run from WSL: the comma device compiles the prebuilt release, WSL pushes it to GitHub.
#
#   DEVICE=comma@192.168.1.50 ./release/remote_release.sh build
#     Clones/updates SRC_BRANCH (default: main-x.xx.x from common/version.h, e.g. main-0.11.0)
#     on the device into /data/openpilot_source and builds the
#     prebuilt into /data/openpilot (replaces what the device currently runs), then
#     restarts openpilot so you can test the exact build before publishing.
#     The prebuilt runs on comma 3 / 3X / four whichever of them builds it. Its branch is named
#     after common/version.h: TOP01100 for "0.11.0.0 DEV. - ..." (set RELEASE_BRANCH to override).
#
#   DEVICE=comma@192.168.1.50 ./release/remote_release.sh push
#     Fetches the built commit from the device and force-pushes it to the same branch
#     name using WSL's GitHub credentials.
set -euo pipefail

DEVICE="${DEVICE:?set DEVICE, e.g. DEVICE=comma@192.168.1.50}"
RELEASE_BRANCH="${RELEASE_BRANCH:-}"  # empty: build_release.sh names it after common/version.h
SRC_REPO="${SRC_REPO:-https://github.com/tto1979/openpilot.git}"
PUSH_REMOTE="${PUSH_REMOTE:-git@github.com:tto1979/openpilot.git}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

# source branch from this checkout's version: "0.11.0.0 DEV. - ..." -> main-0.11.0
VERSION_BRANCH=$(awk -F'"' '/COMMA_VERSION/{split($2,a," "); split(a[1],v,"."); print "main-" v[1] "." v[2] "." v[3]}' common/version.h)
SRC_BRANCH="${SRC_BRANCH:-$VERSION_BRANCH}"
echo "source branch: $SRC_BRANCH"

case "${1:-}" in
  build)
    if ! ssh "$DEVICE" "SRC_BRANCH='$SRC_BRANCH' RELEASE_BRANCH='$RELEASE_BRANCH' SRC_REPO='$SRC_REPO' bash -s" <<'EOF'
source /etc/profile
set -ex
git lfs version > /dev/null || { echo "git-lfs not found on device"; exit 1; }

SRC=/data/openpilot_source
if [ ! -d "$SRC/.git" ]; then
  git clone -b "$SRC_BRANCH" "$SRC_REPO" "$SRC"
fi
cd "$SRC"
git fetch origin "$SRC_BRANCH"
git checkout -f -B "$SRC_BRANCH" "origin/$SRC_BRANCH"
git clean -xdff
git submodule sync --recursive
git submodule update --init --recursive --force
git submodule foreach --recursive "git reset --hard && git clean -xdff"
git lfs pull

sudo systemctl stop comma
RELEASE_BRANCH="$RELEASE_BRANCH" PANDA_DEBUG_BUILD=1 ./release/build_release.sh
sudo systemctl start comma
EOF
    then
      echo "Build failed. openpilot on the device is stopped; fix and rerun 'build', or 'sudo reboot' the device."
      exit 1
    fi
    BUILT_BRANCH=$(ssh "$DEVICE" "git -C /data/openpilot rev-parse --abbrev-ref HEAD")
    echo "Build done ($BUILT_BRANCH), device is running the new build. After testing it, run: $0 push"
    ;;
  push)
    if [ -z "$RELEASE_BRANCH" ]; then
      RELEASE_BRANCH=$(ssh "$DEVICE" "git -C /data/openpilot rev-parse --abbrev-ref HEAD")
    fi
    git fetch "$DEVICE:/data/openpilot" "+refs/heads/$RELEASE_BRANCH:refs/remotes/device/$RELEASE_BRANCH"
    git push -f "$PUSH_REMOTE" "refs/remotes/device/$RELEASE_BRANCH:refs/heads/$RELEASE_BRANCH"
    echo "Pushed: https://github.com/tto1979/openpilot/tree/$RELEASE_BRANCH"
    echo "Install URL: installer.comma.ai/tto1979/$RELEASE_BRANCH"
    ;;
  *)
    echo "usage: DEVICE=comma@<device-ip> $0 build|push"
    exit 1
    ;;
esac
