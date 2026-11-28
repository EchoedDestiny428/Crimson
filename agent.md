# AI Agent Guidelines

> [!CAUTION]
> **EVERYONE MUST BUILD WITH PROS KERNEL 4.2.3 (this means you too, Roy).**
> All programs on a brain share one uploaded library, and it has to be the same build for everyone. If you upload
> with a different kernel (an old local PROS install, an old commit still on 4.2.2, `pros conductor` pulling something
> else), the CLI sees "Library onboard doesn't match!" and **deletes every other slot on the brain** as orphaned.
> This has wiped teammates' slots more than once.
>
> Before any upload:
> 1. Update your PROS CLI / VS Code extension, then pull. The kernel lives in the repo (`firmware/libpros.a`,
>    `include/pros/`, `project.pros` -> `"kernel"` `"version": "4.2.3"`); do not run `pros conductor apply` or
>    `upgrade` to change it on your own.
> 2. Check `include/pros/version.h` says `"4.2.3"`. If your branch says anything else, merge the kernel update in
>    before uploading.
> 3. Run `pros v5 ls-files` and see which slots are on the brain. If you're not sure your library matches, tell the
>    owners of the other slots first.
>
> Changing the kernel version is a team decision: announce it, update this file, and make sure every active branch
> moves at the same time.

The full repository reference (ports, subsystems, autons) is in `agent.md` on the `new-robot` branch.
Every push is checked by `.github/workflows/kernel-check.yml`, which fails if the kernel isn't 4.2.3.
