
# About

This is a solution to the [CartPole-Swing Up problem](https://colab.research.google.com/drive/1MHJp5BsMCGDf2PhbjUfJ4oVdpp0Cm9TY) using a pre-existing iLQR implementation from https://github.com/TGlad/ILQR.

## What I did

* added a `CartPole` class / problem definition, following the example `Acrobot` class / problem definition
   * implemented the dynamics `CartPole::f()`,
     copying from the Python code `CartPole.step()` provided in the assignment
   * implemented the cost functions `CartPole::l()`, `CartPole::lf()`,
     finding a penalization scheme that produces the desired final state

* added `cartpole_main.cpp` that runs the pre-existing iLQR implementation / solver on the above `CartPole` problem
   * copied parameter values (time delta `td`, trajectory length `T`, initial state `x0`) from the assignment code

git diff wrt. fork base:

https://github.com/TGlad/ILQR/compare/22cf187bbc11d29de83eae57d413d0c3823aa687...wiktortomczak:ILQR:master

## CartPole-Swing Up iLQR results visualization

https://user-images.githubusercontent.com/19717465/224421246-a56787c1-e49b-4daf-b0b0-c21ddde9c525.mp4

## How to compile and run

### Compile

optimized

```shell
g++ cartpole_main.cpp -o cartpole_main -Wall -O3
```

debug

```shell
g++ cartpole_main.cpp -o cartpole_main -Wall -O0 -g
```

requires Eigen C++ library (Debian/Ubuntu: `apt install libeigen3-dev`)

### Run

```shell
$ cartpole_main                                # outputs cartpole.trajectory.csv
$ cartpole_render.py cartpole.trajectory.csv   # outputs cartpole.mp4
```

## Files

* `cartpole.h` - CartPole problem definition (dynamics & cost functions)
* `cartpole_main.cpp` - executable, runs iLQR implementation on the above problem
* `cartpole.trajectory.csv` - iLQR output, optimized (state, control) trajectory
* `cartpole.mp4` - visualization of the above trajectory
* `cartpole_render.py` - visualization code, extracted from the assignment code

TODO: cartpole_main static binary

