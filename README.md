# Computer Graphics – 3D Helicopter Model

A 3D helicopter built in **C with OpenGL/GLUT**, demonstrating geometric transformations on a hierarchical model.

**Author:** Rudii
**Subject:** Computer Graphics – college project

## Features
- Helicopter modelled from ellipsoids, cylinders and cubes (fuselage, cockpit glass, side windows, engine cowl, mast, four-blade main rotor, tapered tail boom, fin, stabiliser, tail rotor, landing skids with struts)
- Transformations: translation, rotation (X/Y/Z), uniform and non-uniform scaling, shearing, reflection, composite
- Hierarchical motion: main rotor spin, tail rotor spin, cyclic tilt of the rotor disc
- Planar shadow cast on the helipad using a projection matrix
- Automatic demo of every transformation, with the composite matrix and its parameters shown live
- Ghost wireframe of the untransformed helicopter for before / after comparison

## Build & run
```bash
sudo apt install freeglut3-dev      # Ubuntu / Debian
gcc heli.c -o heli -lglut -lGLU -lGL -lm
./heli
```
macOS: `gcc heli.c -o heli -framework OpenGL -framework GLUT -Wno-deprecated`

Windows (MinGW + freeglut): `gcc heli.c -o heli.exe -lfreeglut -lglu32 -lopengl32`

The same program is also included as `heli.cpp` (C++ version, build with `g++`).

## Controls
| Key | Action |
|-----|--------|
| `SPACE` | start / stop automatic demo (`,` `.` = previous / next step) |
| Arrow keys, PgUp/PgDn | translate X/Z, Y |
| `x` `y` `z` (Shift = reverse) | rotate about X / Y / Z |
| `+` `-` | uniform scale (`j k l` = Sx Sy Sz, Shift = shrink) |
| `u` `i` | shear (Shift = reverse) |
| `7` `8` `9` | reflect about YZ / XZ / XY plane |
| `a` `d` / `f` `b` | cyclic roll / cyclic pitch of the rotor disc |
| `c` | rotors on / off |
| `w` `g` `p` `m` `r` | wireframe, ghost, shadow, print matrices, reset |
| `h` | show / hide help text |
| Mouse drag / wheel, `[` `]` | orbit / zoom |

## How the transformations are applied
The composite matrix is built in code with a small `Mat4` struct and `matMul()` and sent to OpenGL with `glMultMatrixf`, so the matrix shown on screen is exactly the one used for drawing:

```
M = F * T * Ry * Rx * Rz * H * S
```

| Matrix | Meaning |
|--------|---------|
| `S` | scaling (uniform factor x per-axis factors) |
| `H` | shear, `x' = x + k*y` and `y' = y + k*x` |
| `Rz`, `Rx`, `Ry` | rotations about the Z, X and Y axes |
| `T` | translation |
| `F` | reflection, a scale of -1 on the mirrored axis |

Points are transformed right to left. The hierarchy (rotor, tail rotor, tilt) is drawn with `glPushMatrix` / `glPopMatrix`, so each child inherits its parent's matrix.

The shadow uses a separate projection matrix that flattens every vertex onto the ground along the light direction, applied after `M`.
