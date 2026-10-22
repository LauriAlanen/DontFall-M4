# DontFall-M4

Zephyr firmware for the Cortex-M4 coprocessor of the STM32MP157C-DK2.
Linux on the A7 loads it through remoteproc; the two talk over OpenAMP
(rpmsg). The firmware announces one `rpmsg-tty` channel (`/dev/ttyRPMSG0`
on Linux) that carries micro-ROS: the M4 is ROS 2 node `dontfall_m4`
through the micro-ROS Agent on the A7. Its topics and services are listed
in DontFall's `docs/architecture.md`. The orange LED LD7 blinks while the
session with the Agent is up.

| File | Role |
|------|------|
| `app/src/main.c` | wires the modules together |
| `app/src/ipc.c` | OpenAMP/rpmsg transport (rpmsg-tty endpoint) |
| `app/src/link_led.c` | LD7: Agent session up |
| `app/src/uros.c` | micro-ROS node: entities, Agent connection |
| `app/src/uros_transport.c` | micro-ROS custom transport over the rpmsg-tty channel |
| `app/src/diag.c` | `/diagnostics` message |
| `app/src/imu.c` | LSM6DSOX sampling; `imu_health.c` faults, `tilt.c` tilt filter |
| `app/src/control.c` | feeds the supervisor (`supervisor.c`: arm/disarm, cmd_vel timeout) |
| `app/src/wdog.c` | WWDG1 watchdog and its kicker |

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
  modules/lib/micro_ros_zephyr_module
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
only, plus colcon for micro-ROS), runs `west init -l` + `west update` and
applies `patches/`.
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

Host tests (ztest suites in `tests/`, run by twister as Linux programs on
`native_sim/native/64`; results in `build/twister/`):

```sh
scripts/test.sh
```

Only critical logic has tests (e.g. `tests/tilt` for the tilt filter in
`app/src/tilt.c`). Code under test must not depend on Zephyr's drivers,
so the same source builds into the firmware and the test.

micro-ROS: libmicroros is built with colcon inside
`modules/lib/micro_ros_zephyr_module/modules/libmicroros` from the pinned
sources in `micro_ros.yml` (first build: a few minutes). On the board,
`dontfall-uros-agent.service` runs the Agent on `/dev/ttyRPMSG0`; by
hand (stop the service first):

```sh
printf '\n' > /dev/ttyRPMSG0     # Linux must write first, see uros_transport.h
micro-ros-agent serial --dev /dev/ttyRPMSG0
```

and `ros2 topic echo /diagnostics` anywhere on the LAN. The M4 finds a
restarted Agent by itself.

## Patches

`patches/<project>/*.patch` are applied on top of the pinned west projects
by `scripts/apply-patches.sh`. The `dontfall-m4` recipe (Yocto) applies the
Zephyr ones:

- `0001` - don't touch DBGMCU from the M4: with OP-TEE/secure boot an M4
  access to the debug unit stalls the core before `main()`.
- `0002` - backport: the resource table trace entry must point at the RAM
  console buffer, not the pointer to it.

`micro_ros_zephyr_module` (west only until micro-ROS builds in bitbake):

- `0001` - a "custom" transport choice, so the application can provide
  its own transport (the module only has UART/USB/UDP).

## Release

Commit and push, then bump `SRCREV_dfm4` in
`DontFall/meta-df/meta-df-common/recipes-df/dontfall-m4/dontfall-m4.bb`
and rebuild the image.
