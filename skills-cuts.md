# Skills: potential time cuts

Not done yet. Check the section timer (controller readout / `[timer]` lines) before and after each one.

1. **Goal approaches always run their full timeout (~12 s total).** Every slow back-in targets the goal center, so
   the robot stops 7.5 in short against the goal and waits out the timeout. 10 per run: `start` 900 + 650 ms, each
   left loop's curve final leg 1200 ms (x4), `transition` 1300 + 1200 ms, `right_start` 950 + 950 ms.
   - Quick: trim each ~250 ms (~2.5 s).
   - Better: end the move once the robot stalls against the goal (~4-6 s).
2. **Turns settle before the next drive.** ~10 `turnToPoint`s (loader approaches, two in `transition`). An early exit
   (~5 deg) plus `minSpeed` would roll each into its drive: ~150-250 ms each, ~1.5-2.5 s.
3. **1.2 s side toggle spin at the start** holds the robot still. If the toggle can spin while driving, start the
   drive to goal 1 right away: up to 1.2 s.
4. **`transition`'s 72 in back-up** to (132.5, 84) stops before turning to goal 2. Chain it with `minSpeed` and an
   early exit: a few hundred ms.
