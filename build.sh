#! /bin/bash
pushd $(dirname $0) 2>&1 >/dev/null
SRCDIR=$(pwd)
SELF=$(basename $(pwd))
echo srcdir=$SRCDIR
# figure out as suitable PICO SDK
SEARCH_PATH="$PICO_SDK_PATH \
  /opt/pico-sdk-2.2.0 ~/pico-sdk-2.2.0 \
  /opt/pico-sdk-2.1.1 ~/pico-sdk-2.1.1 \
  /opt/pico-sdk-1.5.1 ~/pico-sdk-1.5.1 \
  /opt/pico-sdk ~/pico-sdk"
for d in $SEARCH_PATH; do
  if [ -d $d ]; then
    if [ -f $d/lib/tinyusb/examples/device/board_test/src/tusb_config.h ]; then 
      if [ -f $d/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake ]; then
        SDK_VERSION=2.0
        SDK_PATH=$d
        break
      fi
      if [ -f $d/cmake/preload/toolchains/pico_arm_gcc.cmake ]; then
        SDK_VERSION=1.5
        SDK_PATH=$d
      fi
    else
      echo PICO SDK in $d doesn\'t have TinyUSB - not using.
    fi
  fi
done
if [ -z $SDK_VERSION ]; then
  echo Unable to locate a suitable Pico SDK
  echo \(searched $SEARCH_PATH\)
  popd
  exit
fi

# locate a suitable compiler for cortex-m0
for p in m0 cortexm arm-none-eabi; do
  if which $p-gcc 2>/dev/null >/dev/null; then
    if $p-gcc -mcpu=cortex-m0plus -x c -c /dev/null -o /dev/null 2>/dev/null >/dev/null; then
      M0_PREFIX=$p
      break
    fi
  fi
done
# locate a suitable compiler for cortex-m33
if [ "$SDK_VERSION" = "2.0" ]; then
  echo $p
  for p in m33 cortexm arm-none-eabi; do
    if which $p-gcc 2>/dev/null >/dev/null; then
      if $p-gcc -mcpu=cortex-m33 -x c -c /dev/null -o /dev/null 2>/dev/null >/dev/null; then
        M33_PREFIX=$p
        break
      fi
    fi
  done
fi

if [ -z $M0_PREFIX ]; then
  echo Not attempting to build for Pico.
else
  echo [ building for pico ]
  echo
  mkdir -p bin
  # build the vanilla version
  rm -rf /tmp/${SELF}-build-pico
  mkdir -p /tmp/${SELF}-build-pico
  pushd /tmp/${SELF}-build-pico 
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico -DPICO_PLATFORM=rp2040 -DPICO_GCC_TRIPLE=$M0_PREFIX $SRCDIR
  make -j && \
    cp -v dirtyJtag.uf2 $SRCDIR/bin/dirtyJtag-pico.uf2 && \
    cp -v dirtyJtag.elf $SRCDIR/bin/dirtyJtag-pico.elf && chmod a-x $SRCDIR/bin/dirtyJtag-pico.elf && \
    cp -v git.c $SRCDIR/bin && \
    ${M0_PREFIX}-readelf -a dirtyJtag.elf | tail -12 > $SRCDIR/bin/dirtyJtag-pico.abi
  popd
  echo [ building for pico, usb tty debug ]
  mkdir -p bin
  # build the version that outputs debug info over a ttyACM
  rm -rf /tmp/${SELF}-build-pico-usbdbg
  mkdir -p /tmp/${SELF}-build-pico-usbdbg
  pushd /tmp/${SELF}-build-pico-usbdbg
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico -DPICO_PLATFORM=rp2040 -DPICO_GCC_TRIPLE=$M0_PREFIX -DUSB_DEBUG=y $SRCDIR
  make -j && \
    cp -v dirtyJtag.uf2 $SRCDIR/bin/dirtyJtag-pico-usbdbg.uf2 && \
    cp -v dirtyJtag.elf $SRCDIR/bin/dirtyJtag-pico-usbdbg.elf && chmod a-x $SRCDIR/bin/dirtyJtag-pico-usbdbg.elf
  popd
  echo
fi

if [ -z $M33_PREFIX ]; then
  echo Not attempting to build for Pico2.
else
  echo [ building for pico2 ]
  echo
  rm -rf /tmp/${SELF}-build-pico2
  mkdir -p /tmp/${SELF}-build-pico2
  pushd /tmp/${SELF}-build-pico2
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s -DPICO_GCC_TRIPLE=$M33_PREFIX $SRCDIR
  make -j && \
    cp -v dirtyJtag.uf2 $SRCDIR/bin/dirtyJtag-pico2.uf2 && \
    cp -v dirtyJtag.elf $SRCDIR/bin/dirtyJtag-pico2.elf && chmod a-x $SRCDIR/bin/dirtyJtag-pico2.elf && \
    cp -v git.c $SRCDIR/bin && \
    ${M33_PREFIX}-readelf -a dirtyJtag.elf | tail -16 > $SRCDIR/bin/dirtyJtag-pico2.abi
  popd
  echo [ building for pico2, usb tty debug ]
  echo
  rm -rf /tmp/${SELF}-build-pico2-usbdbg
  mkdir -p /tmp/${SELF}-build-pico2-usbdbg
  pushd /tmp/${SELF}-build-pico2-usbdbg
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s -DPICO_GCC_TRIPLE=$M33_PREFIX -DUSB_DEBUG=y $SRCDIR
  make -j && \
    cp -v dirtyJtag.uf2 $SRCDIR/bin/dirtyJtag-pico2-usbdbg.uf2 && \
    cp -v dirtyJtag.elf $SRCDIR/bin/dirtyJtag-pico2-usbdbg.elf && chmod a-x $SRCDIR/bin/dirtyJtag-pico2-usbdbg.elf
  popd
  echo
fi
popd
