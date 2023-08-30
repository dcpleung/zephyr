# Zephyr with the Xtensa CALL0 ABI

The Xtensa register window mechanism is a poor fit for memory
protection.  The existing Zephyr optimizations, in particular the
cross-stack call we do on interrupt entry, can't be used.  The way
Xtensa windows spill is to write register state through the stack
pointers it finds in the hidden caller registers, which are wholely
under the control of the userspace app that was interrupted.  The
kernel can't be allowed to do that, it's a huge security hole.

Naively, the kernel would have to write out the entire 64-entry
register set on every entry from a context with a PS.RING value other
than zero, including system calls.  That's not really an acceptable
performance or complexity cost.

Instead, for userspace apps, Zephyr builds using the "call0" ABI,
where only a fixed set of 16 GPRs (see section 10 of the Cadence
Xtensa Instruction Set Architecture Summary for details on the ABI).
Kernel traps can then use the remaining GPRs for their own purposes,
greatly speeding up entry speed.

## Toolchain

Existing Xtensa toolchains support a ``-mabi=call0`` flag to generate
code with this ABI, and it works as expected.  It sets a
__XTENSA_CALL0_ABI__ preprocessor flag and the Xtensa HAL code and our
crt1.S file are set up to honor it appropriately (though there's a
glitch in the Zephyr HAL integration, see below).

Unfortunately that doesn't extend to binary artifacts.  In particular
the libgcc.a files generated for our existing SDK toolchains are using
the windowed ABI and will fail at runtime when you hit 64 bit math.

Cadence toolchains have an automatic multilib scheme and will select a
compatible libgcc automatically.

But for now, you have to have a separate toolchain (or at least a
separately-built libgcc) for building call0 apps.  I'm using this
script and it works fine, pending proper SDK integration:

.. code-block:: bash
    #!/bin/sh
    set -ex

    TC=$1
    if [ -z "$TC" ]; then
        TC=sample_controller
    fi

    # Grab source (these are small)
    git clone https://github.com/zephyrproject-rtos/sdk-ng
    git clone https://github.com/crosstool-ng/crosstool-ng

    # Build ct-ng itself
    cd crosstool-ng
    ./bootstrap
    ./configure --enable-local
    make -j$(nproc)

    # Configure for the desired toolchain
    ln -s ../sdk-ng/overlays
    cp ../sdk-ng/configs/xtensa-${TC}_zephyr-elf.config .config

    grep -v CT_TARGET_CFLAGS .config > asdf
    mv asdf .config
    echo CT_TARGET_CFLAGS="-mabi=call0" >> .config
    echo CT_LIBC_NONE=y >> .config

    ./ct-ng olddefconfig
    ./ct-ng build.$(nproc)

    echo "##"
    echo "## Set these values to enable this toolchain:"
    echo "##"
    echo export CROSS_COMPILE=$HOME/x-tools/xtensa-${TC}_zephyr-elf/bin/xtensa-${TC}_zephyr-elf-
    echo export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile

Note you don't really need to use the toolchain that was built, it's
enough to take the libgcc.a and drop it on top of the one in your SDK,
just be careful to save the original, etc...

Longer term we need proper integration into the SDK, obviously.
Ideally we would just rebuild libgcc in isolation, but I couldn't
figure out how to do that except in the context of a full gcc build.
We could do the separate build in sdk-ng and then copy the files to a
multilib scheme of our own devising, which is something existing
Zephyr platforms sometimes need to do.  Also I note that other gcc
architectures have already solved problems like this (c.f. x86_64 and
x32 working in the same toolchain, etc...), but couldn't find any
Xtensa-specific support.

Also as a quick start: it's possible to build Zephyr in such a way as
to avoid (almost/maybe) all calls to libgcc via disabling kconfigs
that trigger 64 bit code.  I found I could get full coverage of the
new call0 code with a custom test rig and ``-- -DCONFIG_TIMEOUT_64BIT=n -DCONFIG_CBPRINTF_REDUCED_INTEGRAL=y -DCONFIG_CBPRINTF_NANO=y``

## Bugs and Missing Features

No support in the SDK.  See above about toolchains.

The existing syscall implementation is a mock, because this happened
in parallel with the new MMU-enabled dc233c platform.  The only code
difference is that it uses a single static region loaded from a
"_mock_priv_stack" symbol instead of a real thread kernel stack.  In
theory everything will work when you enable CONFIG_USERSPACE, but...

Likewise the MMU integration is a blindly-copied stub and unexercised.
It's really simple though ("just read from PTEVADDR with PS.EXCM=1 to
prime the TLB"), so hopefully it won't need much fixing.

There's no C implementation of the Zephyr "call syscalls" API, just
assembly.  The interface is deliberately based on the call0 function
call ABI though, so this should be trivially easy.

For simplicity, this code is written to save context to the arch
region of the thread struct and not the stack (except for nested
interrupts, obviously).  That's a common pattern in Zephyr, but for
KERNEL_COHERENCE (SMP) platforms it's actually a performance headache,
because the stack is cached where the thread struct is not.  We should
move this back to the stack, but that requires doing some logic in the
assembly to check that the resulting stack pointer doesn't overflow
the protected region, which is slightly non-trivial (or at least needs
a little inspiration).

Right now the Zephyr HAL integration doesn't build when the call0 flag
is set because of a duplicated symbol.  I just disabled the build at
the cmake level for now, but this needs to be figured out.

The ARCH_EXCEPT() handling is a stub and doesn't actually do anything.
Really this isn`t any different than the existing code, it just lives
in the asm2 files that were disabled and I have to find a shared
location for it.

Backtrace logging is likewise specific to the older frame format and
ABI and doesn't work.  The call0 ABI is actually much simpler, though,
so this shouldn't be hard to make work.

FPU support isn't enabled.  Likewise this isn't any different (except
trivially -- the context struct has a different layout).  We just need
to copy code and find compatible hardware to test it on.

A good security audit is needed to make sure all the holes are
plugged.  At the very least, note that when the PS.WOE bit is set,
user thread contexts are able to "rotate" registers and write to their
contexts by doing regular function calls.  We need to be 100% sure no
RING 1+ code ever runs with this set.  (Also the ISA spec is a little
ambiguous as to how the hardware treats CALLn/ENTRY/RETW instructions
when WOE=0, I worry a tiny bit there might be hardware that has
unpluggable holes...)

I realized when writing this that our existing handling for NMI
exceptions (strictly any level > EXCM_LEVEL) isn't correct, as we
generate ZSR-style EPS/EPC register usage in those handlers that isn't
strictly separated from the code it may/will have interrupted.  This
is actually a bug with asm2 also, but it's going to make
high-priority/DEBUG/NMI exceptions unreliable unless fixed (I don't
think there are any Zephyr users currently though).
