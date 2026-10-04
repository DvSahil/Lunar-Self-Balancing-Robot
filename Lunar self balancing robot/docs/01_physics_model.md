# 1. Physics and model

## Why a balancing robot needs a controller at all

Treat the robot as an **inverted pendulum on a cart**: a body (mass `m`, centre of mass a height `l`
above the wheel axle) balanced on wheels (mass `M`) that can only move forward and backward.

If the body tilts forward by a small angle θ, gravity pulls it further forward. The farther it
leans, the harder gravity pulls. This is **positive feedback**, so the upright position is
*unstable*. The only way to fix a forward lean is to **drive the wheels forward, under the centre
of mass**, faster than the body falls. A controller does exactly that, hundreds of times per second.

## Equations of motion

With θ measured from upright (+ = leaning forward), `x` the wheel position and `F` the force the
wheels push on the ground:

```
(M + m) x'' + m l cosθ · θ'' − m l sinθ · θ'² = F − b x'
(I + m l²) θ''  + m l cosθ · x''  − m g l sinθ = 0
```

* `I`  inertia of the body about its own centre of mass (a uniform bar of height `h`: `m h² / 12`)
* `b`  viscous friction of the drivetrain

Read the second line as a story: gravity (`m g l sinθ`) tries to increase θ, and accelerating the
wheels (`x''`) reduces θ''. That coupling is the *only* handle the controller has.

## Linearisation (small angles)

For |θ| below about 10°, `sinθ ≈ θ`, `cosθ ≈ 1` and the θ'² term disappears:

```
[M+m   m l   ] [x'' ]   [ F − b x' ]
[m l  I+m l² ] [θ'' ] = [ m g l θ  ]
```

In state-space form `s' = A s + B F` with `s = [x, x', θ, θ']`. `simulation/pendulum_sim.py` builds
`A` and `B` numerically. Its open-loop poles come out near **+14 and −14 rad/s**. The positive pole is the
instability, and it tells you how fast the controller must be: a time constant of about
1/14 ≈ 70 ms means a loop period of 5 ms (200 Hz) gives you roughly 14 samples per time constant.

## What the controller can and cannot do

* A controller on **tilt only** (PD) can keep the robot upright, but it has no idea where the robot
  *is*, so the robot slowly rolls away. The simulation shows exactly this
  (`docs/images/sim_comparison.png`, blue curve).
* To also hold position or speed you need feedback from the wheel encoders. In steady state a robot
  that leans forward by θ accelerates forward at about `g·tanθ`, so **to slow down you lean backward
  first**. That is why the outer loop changes the *angle target* instead of driving the motors
  directly (cascade control, see doc 4).
* **LQR** computes all four feedback gains at once from the linear model. It is optimal for the
  model you give it, but needs a decent model (mass, centre of mass, motor constants).

## Measure your own parameters

| Parameter | How to get it |
|-----------|---------------|
| `m`, `M` | Weigh the body and the wheel/motor assemblies on a kitchen scale |
| `l` | Hang or balance the body on a knife edge to find the centre of mass, measure its height above the axle |
| wheel radius | Calipers |
| force limit | Motor stall torque (datasheet) × 2 motors ÷ wheel radius, then derate by about 50% for the battery voltage and driver losses |

Put your numbers at the top of `simulation/pendulum_sim.py` and compare the simulated response with
your logged one. The comparison is a good thing to write in the README.
