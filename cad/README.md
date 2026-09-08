# CAD

`step/actuator_assembly.step` is the actuator as built.

`step/alternates/rev0_design.step` is the design before assembly.

## What changed

Two parts came out between them: a POM bushing on a bracket supporting the
output stub, and the bearing nut that clamps the top bearing's inner race.

The bushing was rigid and over-constrained a stub that is already located by
the output flange at one end.

The bearing nut came out because of assembly order, not design. The piece that
was designed to clamp the top bearing's inner race from below was unfortunately
inaccessible in the reversed assembly order, so I had to remove it. Currently,
the inner race is held by the shaft shoulder in the opposite direction of the
axial thrust, so the only thing holding the inner race in place is the Loctite
retaining compound. The strain wave cup thrusts the wave generator toward the
open end every revolution, and that load pulls the race away from the shoulder.
I knew this early on, and without retention, the wave generator physically
walked out during a calibration spin. The Orbis LED went yellow since the
readhead gap had shifted. While Loctite 641 technically has a shear strength
stronger than what the axial thrust from the gearbox exerts, it's obviously not
an ideal solution, and is the first part of the design I would fix if I made a
version 2.

## Part names

A few components carry the name I gave them while building rather than what
they are.

`Output Shaft` is the output flange and the 5 mm stub that runs down through
the bore, as one part. `Shaft` is the motor shaft. `Alu Large Shaft` is the
rotor hub. `TopBearingCover` holds the top fixed bearing and `16287 Bearing
Lip` retains it. `RI60 Cup` is the motor housing and `3Dprinted Case` is the
electronics enclosure at the bottom. `backiron_washer` is the 2 mm steel washer
under the output magnet. Spacers are 8 mm M2.5 under the moteus and 1 mm M2
under the Orbis.

Bought parts keep their part numbers: `F17(8轴)STEP` is the ZXF17-50 gearbox,
`RI60-210120` the CubeMars motor, `BR10xxx14xxxxD00` and `BM120A190A1ABA00` the
RLS Orbis readhead and ring magnet, `6705zz` the floating bottom bearing,
`bearing_16x28x7_2RS_with_balls` the fixed top one, and `6mm magnet` the
diametric magnet on the output stub. Anything called `Part 1` through `Part 38`
is a fastener or gearbox internals.
