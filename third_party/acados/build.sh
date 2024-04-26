#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

ARCHNAME="x86_64"
BLAS_TARGET="X64_AUTOMATIC"
if [ -f /TICI ]; then
  ARCHNAME="larch64"
  BLAS_TARGET="ARMV8A_ARM_CORTEX_A57"
elif [ -f /EON ]; then
  ARCHNAME="aarch64"
  BLAS_TARGET="ARMV8A_ARM_CORTEX_A57"
fi

ACADOS_FLAGS="-DACADOS_WITH_QPOASES=ON -UBLASFEO_TARGET -DBLASFEO_TARGET=$BLAS_TARGET"

if [[ "$OSTYPE" == "darwin"* ]]; then
  ACADOS_FLAGS="$ACADOS_FLAGS -DCMAKE_OSX_ARCHITECTURES=arm64;x86_64 -DCMAKE_MACOSX_RPATH=1"
  ARCHNAME="Darwin"
fi

if [ ! -d acados_repo/ ]; then
  git clone https://github.com/acados/acados.git $DIR/acados_repo
  # git clone https://github.com/commaai/acados.git $DIR/acados_repo
fi
cd acados_repo
git fetch --all
git checkout 8ea8827fafb1b23b4c7da1c4cf650de1cbd73584
git submodule update --recursive --init

# build
mkdir -p build
cd build
cmake $ACADOS_FLAGS ..
make -j20 install

INSTALL_DIR="$DIR/$ARCHNAME"
rm -rf $INSTALL_DIR
mkdir -p $INSTALL_DIR

rm $DIR/acados_repo/lib/*.json

rm -rf $DIR/include $DIR/acados_template
cp -r $DIR/acados_repo/include $DIR
cp -r $DIR/acados_repo/lib $INSTALL_DIR
cp -r $DIR/acados_repo/interfaces/acados_template/acados_template $DIR/
#pip3 install -e $DIR/acados/interfaces/acados_template

# hack to workaround no rpath on android
if [ -f /EON ]; then
  pushd $INSTALL_DIR/lib
  for lib in $(ls .); do
    if ! readlink $lib; then
      patchelf --set-soname $PWD/$lib $lib

      if [ "$lib" = "libacados.so" ]; then
        for nlib in "libhpipm.so" "libblasfeo.so" "libqpOASES_e.so.3.1"; do
          patchelf --replace-needed $nlib $PWD/$nlib $lib
        done
      fi

      if [ "$lib" = "libhpipm.so" ]; then
        patchelf --replace-needed libblasfeo.so $PWD/libblasfeo.so $lib
      fi

      # pad extra byte to workaround bionic linker bug
      # https://android.googlesource.com/platform/bionic/+/93ce35434ca5af43a7449e289959543f0a2426fa%5E%21/#F0
      dd if=/dev/zero bs=1 count=1 >> $lib
    fi
  done
  popd

  cd $DIR
  git checkout $INSTALL_DIR/t_renderer
else
# build tera
cd $DIR/acados_repo/interfaces/acados_template/tera_renderer/
if [[ "$OSTYPE" == "darwin"* ]]; then
  cargo build --verbose --release --target aarch64-apple-darwin
  cargo build --verbose --release --target x86_64-apple-darwin
  lipo -create -output target/release/t_renderer target/x86_64-apple-darwin/release/t_renderer target/aarch64-apple-darwin/release/t_renderer
else 
  cargo build --verbose --release
fi
cp target/release/t_renderer $INSTALL_DIR/

fi
