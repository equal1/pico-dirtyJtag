#! /bin/bash
pushd $(dirname $0) 2>&1 >/dev/null
SELF=$(basename $(pwd))
# figure out as suitable PICO SDK
SEARCH_PATH="$PICO_SDK_PATH \
  /opt/pico-sdk-2.1.0 ~/pico-sdk-2.1.0 \
  /opt/pico-sdk-2.0.0 ~/pico-sdk-2.0.0 \
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
  rm -rf ../${SELF}-build-pico
  mkdir -p ../${SELF}-build-pico
  cd ../${SELF}-build-pico 
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico -DPICO_PLATFORM=rp2040 -DPICO_GCC_TRIPLE=$M0_PREFIX ../$SELF
  make -j && \
    cp -v dirtyJtag.uf2 ../${SELF}/bin/dirtyJtag-pico.uf2 && \
    cp -v dirtyJtag.elf ../${SELF}/bin/dirtyJtag-pico.elf && chmod a-x ../${SELF}/bin/dirtyJtag-pico.elf && \
    ${M0_PREFIX}-readelf -a dirtyJtag.elf | tail -12 > ../${SELF}/bin/dirtyJtag-pico.abi
  cd ../${SELF}
  echo
fi

if [ -z $M33_PREFIX ]; then
  echo Not attempting to build for Pico2.
else
  echo [ building for pico2 ]
  echo
  rm -rf ../${SELF}-build-pico2
  mkdir -p ../${SELF}-build-pico2
  cd ../${SELF}-build-pico2
  cmake -DPICO_SDK_PATH=$SDK_PATH -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s -DPICO_GCC_TRIPLE=$M33_PREFIX ../$SELF
  make -j && \
    cp -v dirtyJtag.uf2 ../${SELF}/bin/dirtyJtag-pico2.uf2 && \
    cp -v dirtyJtag.elf ../${SELF}/bin/dirtyJtag-pico2.elf && chmod a-x ../${SELF}/bin/dirtyJtag-pico2.elf && \
    ${M33_PREFIX}-readelf -a dirtyJtag.elf | tail -16 > ../${SELF}/bin/dirtyJtag-pico2.abi
  cd ../${SELF}
  echo
fi
popd
