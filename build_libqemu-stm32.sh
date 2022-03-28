#!/bin/sh

set -e

target="arm-softmmu"

case "$(uname)" in
  Linux)
    ncpu="$(nproc)"
    ;;
  Darwin)
    ncpu="$(sysctl -n hw.physicalcpu)"
    ;;
esac

flags="$(./configure --help | perl -ne 'print if s/^  ([a-z][\w-]*) .*/\1/' | tail -n +2 | awk '{print "--disable-"$1}' ORS=' ')"

${2:-.}/configure --target-list=$target --extra-cflags=-fPIC --disable-slirp $flags --enable-tcg \
	--enable-system --disable-werror --disable-alsa  \
        --enable-debug --enable-debug-tcg --enable-debug-info 

make clean >/dev/null

# Build everything as usual
make "-j$ncpu" 

# Build a shared library, without softmmu/main.o and otherwise *exactly* the same
# flags.
cd build
rm -f qemu-system-arm
ninja -d keeprsp
cp qemu-system-arm.rsp qemu-system-arm-static.rsp
#dynamic
sed -i 's/qemu-system-arm.p\/softmmu_main.c.o//g' qemu-system-arm.rsp
sed -i 's/-o\ qemu-system-arm/-shared\ -o\ libqemu-stm32.so/g' qemu-system-arm.rsp
c++ -m64 -mcx16 -ggdb @qemu-system-arm.rsp
#static
sed -i 's/qemu-system-arm.p\/softmmu_main.c.o//g' qemu-system-arm-static.rsp
sed -i 's/-o\ qemu-system-arm//g' qemu-system-arm-static.rsp
sed -i 's/-Wl,--whole-archive//g' qemu-system-arm-static.rsp
sed -i 's/-Wl,--no-whole-archive//g' qemu-system-arm-static.rsp 
sed -i 's/-Wl,--warn-common//g' qemu-system-arm-static.rsp 
sed -i 's/-Wl,-z,relro//g' qemu-system-arm-static.rsp 
sed -i 's/-Wl,-z,now//g' qemu-system-arm-static.rsp 
sed -i 's/-fstack-protector-strong//g' qemu-system-arm-static.rsp 
sed -i 's/-fPIC//g' qemu-system-arm-static.rsp 
sed -i 's/-Wl,--start-group//g' qemu-system-arm-static.rsp
sed -i 's/-Wl,--as-needed//g' qemu-system-arm-static.rsp 
sed -i 's/-Wl,--no-undefined//g' qemu-system-arm-static.rsp
sed -i 's/-Wl,--end-group//g' qemu-system-arm-static.rsp
sed -i 's/@qemu.syms//g' qemu-system-arm-static.rsp
sed -i 's/@block.syms//g' qemu-system-arm-static.rsp
sed -i 's/-lutil//g' qemu-system-arm-static.rsp
sed -i 's/-lm//g' qemu-system-arm-static.rsp 
sed -i 's/-pthread//g' qemu-system-arm-static.rsp 
sed -i 's/-lgthread-2.0//g' qemu-system-arm-static.rsp 
sed -i 's/-lglib-2.0//g' qemu-system-arm-static.rsp 
sed -i 's/-lstdc++//g' qemu-system-arm-static.rsp 
sed -i 's/-lfdt//g' qemu-system-arm-static.rsp

ar rcvs libqemu-stm32.a @qemu-system-arm-static.rsp



