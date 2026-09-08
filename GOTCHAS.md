# Things that cost me days

Mostly lifted from the writeup, which has the full story. This is the short
version, in rough order of how much time each one took.

## The query reply truncates silently

A moteus query reply has to fit in one 64-byte CAN-FD frame. Ask for one
register too many and the firmware does not error, it drops the tail. The first
casualty is FAULT, which is the register every abort check reads. So the safety
check goes blind and reports nothing, and everything looks fine right up until
it is not.

`verify_query()` in `actuator/link.py` asserts the reply came back whole. Run
`--dry-run` after touching the register list, every time.

## kp does not survive a gear ratio

The moteus's default gains, which are tuned for a bare motor, were 2500x too
soft through the 50:1 gearbox (stiffness reflects through a gear ratio as the
square of the ratio), and a twenty minute automated gain sweep found kp at
8000 which was 2x margin under the flexspline's own resonance.

## 6.2 degrees of magnet rotation is 87 degrees of electrical angle

My replacement ring magnet was mounted 6.2° rotated from the original, which
through 14 pole pairs is 87° of electrical angle, so the controller was
delivering the exact current I commanded at exactly the wrong angle. I got 6% of
my torque, telemetry reported perfect tracking, and from the outside it looked
like a broken gearbox. All of which was fixed by recalibration.

Recalibrate after touching the magnets. Always.

## A full day lost to BiSS-C wiring

There was also a full day lost to incorrectly soldering the BiSS-C wiring and
it was completely because I read some docs wrong.

## Two magnetic encoders a few mm apart interfere

There's one problem with parking two magnetic encoders a couple millimeters
apart: their magnetic fields disturb each other. The diametric magnet on the
output stub is nested directly inside the Orbis ring, and while it is just
below the sensor chips, the magnetic field from the ring is disturbed. To fix
this, I added a 2 mm steel washer glued to the shaft right below the diametric
magnet acting as a back-iron, giving the stray flux a return path instead of a
route to the wrong sensor. I measured the interference with the washer off and
on and with it the AS5047P's jitter actually beats a bare magnet (6 counts vs
9). The residual disturbance on the Orbis is ±66 counts, about ±20° electrical.
Not the best but it's only a 6% torque ripple with the output angle, well clear
of the level where commutation starts to degrade.

`bringup/magnet_check.py` measures this. It is only valid with the output
disconnected from the gearbox, so the magnet can turn while the rotor stays
still. Assembled, turning the output turns the rotor 50x with it and you cannot
separate the two effects.

## Detents are not friction, and a velocity gate will not touch them

The friction map came from slow constant speed sweeps by logging torque against
absolute rotor phase. Feed 80% of that map forward (100% risks driving the
joint since kinetic friction at the loose spots is lower than the actual map)
gated by velocity so it only pushes when the joint moves, and the powered joint
gets much smoother. But the hand-felt notches didn't budge, and figuring out
why took a decent amount of time. When I logged where my hand released the arm,
every rest position was at two or three specific rotor phases, and that's not
due to friction. These spots felt like magnetic pulls, and they are detents, a
conservative torque with preferred parking spots, acting at zero velocity,
which is why any velocity gated compensation didn't work. I was focusing on the
wrong problem.

Extracting the detent field was simple. Friction flips sign with direction and
a conservative torque doesn't, so the average of the two directions' torque
profiles is the detent map. Applied always-on with no velocity gate since
detents don't have one.

## Then I counted the detent field twice

The detents had one more farewell bug. My impedance controller returned crisply
in one direction and sluggishly in the other, so I fit separate friction maps
per direction and it made it worse, mainly because the difference between the
two directions' maps was once again, the detent field, which I was already
compensating. Friction on this gearbox is almost symmetric and I was double
counting the field with opposite signs. Subtract it once, use one averaged map,
and the asymmetry was due to me.

## Feedforward at 100% drives the joint by itself

If you push the compensation to 100% and the joint drives itself, since the
friction map is built from data while the joint was moving, and kinetic
friction at the loose spots near zero speed is lower than anything else in the
map, so somewhere there's an angle where the feedforward wins. 80% with damping
is a better ceiling.

## Assembly order is part of the design

I had built my motor and shaft as one finished unit, with the wave generator
resting on the shaft, planning to push it in from the bottom side. It does not
go, and it was never going to. While technically being the same hole, same
diameter, same dimensions, the bottom of the flexspline does not deform.

Reversing the order cost me twice: the ring magnet, already epoxied into the
rotor hub, and the bearing nut, which became unreachable. `cad/README.md` has
the details.

## What software could not fix

I want to be honest about what the software did and didn't fix. Software turned
a notchy, eccentric, 3 piece input stack into a joint that is guided smoothly
by a user. That's the software accounting for the hardware flaws. What software
couldn't buy is free-hand precision. Sensing force through the flexspline
windup needs 0.05° out of an encoder pair with 0.29° of noise, mainly dominated
by the once-per-rev wobble from the tilted washer under the output magnet. The
signal sits six times below the floor, and isn't fixable by any firmware.

## The controller really was broken, once

Worth saying because I spent a while assuming it was me. Midway through
testing, the moteus began raising fault 33 milliseconds into current mode with
its gate-driver fault registers reading clean. The board browned out before the
driver could find anything. I tested to see if it was a PSU failure by plugging
a controller to a 12 V battery and the driver showed the real fault. It was a
genuine gate driver failure, phase C high side, and was apparently due to a
firmware bug that misconfigured the gate driver, leaving it vulnerable to
damage. Mine died even at 24 V. mjbots quickly replaced it.

If the fault registers are empty, suspect your power supply before you suspect
the register.
