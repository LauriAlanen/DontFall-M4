# DontFall-M4

Zephyr firmware for the Cortex-M4 coprocessor of the STM32MP157C-DK2.
Linux on the A7 loads it through remoteproc; the two talk over OpenAMP
(rpmsg). The firmware announces an `rpmsg-tty` channel (`/dev/ttyRPMSG0` on
Linux) and speaks a small line protocol on it (`app/src/protocol.h`): the
A7 sends `PING <seq>` heartbeats, the M4 answers `PONG <seq>`. While
heartbeats keep arriving the orange LED LD7 blinks; after 1.5 s without one
it goes off. A blinking LD7 proves remoteproc, rpmsg and the A7 app all work.

| File | Role |
|------|------|
| `app/src/main.c` | wires the modules together |
| `app/src/ipc.c` | OpenAMP/rpmsg transport (rpmsg-tty endpoint) |
| `app/src/protocol.c` | line assembly and command dispatch |
| `app/src/heartbeat.c` | A7 liveness watchdog, drives LD7 |

Memory map (`app/boards/stm32mp157c_dk2.overlay`): code in MCU SRAM1
(128 KB), data in SRAM2 (128 KB), rpmsg buffers in SRAM3, and only the
vector table at address 0 in RETRAM, where the M4 boots from
(`CONFIG_ROMSTART_RELOCATION_ROM`).

Release builds come from the [DontFall](https://github.com/LauriAlanen/DontFall)
Yocto image (`dontfall-m4` recipe, meta-zephyr multiconfig). Day to day you
build and load it with west, without bitbake.

Full workflow across all DontFall repos (setup, dev loops, release):
[DontFall/docs/development.md](https://github.com/LauriAlanen/DontFall/blob/main/docs/development.md).

## Workspace layout

This repo is the manifest repo of a west workspace:

```
~/Projects/DontFall-M4-ws/     west workspace (not a git repo)
  .west/
  DontFall-M4/                 this repo
  zephyr/                      pinned by west.yml
  modules/hal/{cmsis,stm32,libmetal}, modules/lib/open-amp
```

`west.yml` pins Zephyr to exactly the commit meta-zephyr (scarthgap) builds
in the image. `scripts/check-zephyr-pin.sh` compares the two when
`~/Projects/DontFall` is checked out; bump both together.

## Setup (once)

```sh
mkdir -p ~/Projects/DontFall-M4-ws
git clone git@github.com:LauriAlanen/DontFall-M4.git ~/Projects/DontFall-M4-ws/DontFall-M4
~/Projects/DontFall-M4-ws/DontFall-M4/scripts/init.sh
```

`init.sh` builds the `dontfall-zephyr` container (Zephyr SDK 0.16.9, arm
only), runs `west init -l` + `west update` and applies `patches/zephyr/`.
Re-run it after changing `west.yml`.

## Develop

```sh
scripts/build.sh                    # -> build/zephyr/zephyr.elf
scripts/deploy.sh <board-ip> --log  # load on the running board, show M4 log
```

`deploy.sh` copies the ELF to `/lib/firmware/dontfall-m4.elf` (the name the
image ships, so it replaces the packaged firmware until the next image
deploy), restarts the M4 via `/sys/class/remoteproc/remoteproc0` and with
`--log` prints the M4's `printk` output (RAM console exposed as remoteproc
`trace0`). Extra `build.sh` args go to `west build` (`-p always` = pristine).

Talk to it from the board:

```sh
stty -F /dev/ttyRPMSG0 raw -echo    # no tty echo, or replies bounce back
cat /dev/ttyRPMSG0 & echo "PING 1" > /dev/ttyRPMSG0   # -> PONG 1, LD7 blinks for 1.5 s
```

Stop `dontfall-app.service` first, or it competes for the tty.

## Zephyr patches

`patches/zephyr/*.patch` are applied on top of the pinned Zephyr by
`scripts/apply-patches.sh` (west) and by the `dontfall-m4` recipe (Yocto):

- `0001` - don't touch DBGMCU from the M4: with OP-TEE/secure boot an M4
  access to the debug unit stalls the core before `main()`.
- `0002` - backport: the resource table trace entry must point at the RAM
  console buffer, not the pointer to it.

## Release

Commit and push, then bump `SRCREV_dfm4` in
`DontFall/meta-df/meta-df-common/recipes-df/dontfall-m4/dontfall-m4.bb`
and rebuild the image.
