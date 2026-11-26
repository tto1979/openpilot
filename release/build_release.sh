#!/usr/bin/env bash
set -e
set -x

# git diff --name-status origin/release3-staging | grep "^A" | less

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

cd $DIR

BUILD_DIR=/data/openpilot
SOURCE_DIR="$(git rev-parse --show-toplevel)"

# prebuilt must be compiled on the comma device (larch64 + Adreno GPU for tinygrad DEV=QCOM)
if [ ! -f /TICI ]; then
  echo "build_release.sh must run on a comma device. From WSL use release/remote_release.sh"
  exit 1
fi

# TOP: one prebuilt per device, named after the device it's compiled on
#   comma 3 (tici):  version "0.11.0.0 DEV._C3 - ...", branch TOP01100-C3, "For C3 Only, ..."
#   comma 3X (tizi): version "0.11.0.0 DEV. - ...",    branch TOP01100,    "For C3X Only, ..."
DEVICE_TYPE=$(tr -d '\0' < /sys/firmware/devicetree/base/model | sed 's/.*comma //')
case "$DEVICE_TYPE" in
  tici) DEVICE_SUFFIX="C3"; DEVICE_NAME="C3" ;;
  tizi) DEVICE_SUFFIX="";   DEVICE_NAME="C3X" ;;
  mici) DEVICE_SUFFIX="C4"; DEVICE_NAME="C4" ;;
  *) echo "unknown device type: $DEVICE_TYPE"; exit 1 ;;
esac

# add the device suffix to the version's second word ("DEV." -> "DEV._C3") unless it already has one
version_h_with_suffix() {
  if [ -n "$DEVICE_SUFFIX" ]; then
    sed -E "s/^(#define COMMA_VERSION \"[^ ]+ [^ _\"]+) /\1_${DEVICE_SUFFIX} /" "$1"
  else
    cat "$1"
  fi
}
# a version already marked for another device would name the branch after that device
VERSION_SUFFIX=$(grep COMMA_VERSION "$SOURCE_DIR/common/version.h" | awk -F'"' '{split($2,a," "); if (index(a[2],"_")) {s=a[2]; sub(/.*_/,"",s); print s}}')
if [ -n "$VERSION_SUFFIX" ] && [ "$VERSION_SUFFIX" != "$DEVICE_SUFFIX" ]; then
  echo "common/version.h is marked _$VERSION_SUFFIX but this is a $DEVICE_NAME ($DEVICE_TYPE), remove the suffix from version.h"
  exit 1
fi
VERSION_H="$(version_h_with_suffix "$SOURCE_DIR/common/version.h")"
VERSION=$(echo "$VERSION_H" | grep COMMA_VERSION | awk -F'"' '{print $2}' | awk -F'(' '{print $1}' | sed 's/ *$//')
MODEL=$(echo "$VERSION_H" | grep COMMA_VERSION | awk -F'"' '{print $2}' | awk -F'[()]' '{print $2}')
BRANCH=$(echo "$VERSION_H" | grep COMMA_VERSION | awk -F'"' '{split($2,a," "); v=a[1]; gsub(/[^0-9]/,"",v); s=""; if (index(a[2],"_")) {s=a[2]; sub(/.*_/,"",s); s="-"s}; print v s}')
RELEASE_BRANCH="${RELEASE_BRANCH:-TOP$BRANCH}"
COMMIT_TITLE="For $DEVICE_NAME Only, Update to T.O.P v$VERSION"
echo "[-] $DEVICE_TYPE: $RELEASE_BRANCH, \"$COMMIT_TITLE\" / \"$MODEL\""


# set git identity
source $DIR/identity.sh

echo "[-] Setting up repo T=$SECONDS"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
git init
git remote add origin ${RELEASE_REMOTE:-https://github.com/tto1979/openpilot.git}
git checkout --orphan $RELEASE_BRANCH

# do the files copy
echo "[-] copying files T=$SECONDS"
cd $SOURCE_DIR
# files_common adds back NNFF lat_models, which release_files.py blacklists
cp -pR --parents $(./release/release_files.py; cat release/files_*) $BUILD_DIR/

# in the directory
cd $BUILD_DIR

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed
rm -f panda_tici/board/obj/panda.bin.signed
rm -f panda_tici/board/obj/panda_h7.bin.signed

# make sure LFS objects were pulled, not just pointer files (models, fonts, icons, sounds, acados libs)
LFS_POINTERS="$(find . -type f -not -path './.git/*' -size -200c -exec grep -l '^version https://git-lfs.github.com/spec/v1' {} + || true)"
if [ -n "$LFS_POINTERS" ]; then
  echo "LFS pointer files instead of their content, run 'git lfs pull' in $SOURCE_DIR first:"
  echo "$LFS_POINTERS"
  exit 1
fi

echo "$VERSION_H" > common/version.h

echo "[-] committing version $VERSION T=$SECONDS"
git add -f .
git commit -a -m "$COMMIT_TITLE" -m "$MODEL"

# Build
export PYTHONPATH="$BUILD_DIR"
scons -j$(nproc) --minimal

if [ -z "$PANDA_DEBUG_BUILD" ]; then
  # release panda fw
  CERT=/data/pandaextra/certs/release RELEASE=1 scons -j$(nproc) panda/ panda_tici/
else
  # build with ALLOW_DEBUG=1 to enable features like experimental longitudinal
  scons -j$(nproc) panda/ panda_tici/
fi

# Ensure no submodules in release
# (git submodule--helper list was removed in git 2.38)
if [ -n "$(git ls-files -s | awk '$1 == "160000"')" ]; then
  echo "submodules found:"
  git ls-files -s | awk '$1 == "160000"'
  exit 1
fi
git submodule status

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
# modeld only loads the compiled tinygrad pkls
rm -f selfdrive/modeld/models/*.onnx

find third_party/ -name '*x86*' -exec rm -r {} +
find third_party/ -name '*Darwin*' -exec rm -r {} +


# Restore third_party
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .
git commit --amend -m "$COMMIT_TITLE" -m "$MODEL"

# GitHub rejects files over 100MB
BIG_FILES="$(find . -type f -not -path './.git/*' -size +95M)"
if [ -n "$BIG_FILES" ]; then
  echo "Found files exceeding GitHub's 100MB limit:"
  echo "$BIG_FILES"
  exit 1
fi

# Run tests (opt-in)
if [ -n "$RUN_TESTS" ]; then
  cd $BUILD_DIR
  RELEASE=1 pytest -n0 -s selfdrive/test/test_onroad.py
fi

# Push from device (opt-in). Default flow pushes from WSL via release/remote_release.sh
if [ -n "$PUSH" ]; then
  echo "[-] pushing release T=$SECONDS"
  git push -f origin $RELEASE_BRANCH:$RELEASE_BRANCH
fi

echo "[-] done T=$SECONDS"
