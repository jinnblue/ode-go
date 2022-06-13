ode
===

This is a Go binding for the Open Dynamics Engine 3D physics library.  It
sticks fairly closely to the development version of the ODE C API, with a few
stylistic and idiomatic changes thrown in here and there where it seemed
useful.

Get ODE [here](http://bitbucket.org/odedevs/ode/).

ODE must be compiled as a shared library with double precision support.
Triangle mesh indices are expected to be 32 bit, which is the ODE default.  The
following will configure ODE with these options:

`> cd /path/to/ode-src; ./configure --enable-double-precision --enable-shared`


build
===
1. git clone this project in `xxxpath/ode-go`
2. git clone ode project in `xxxpath/ode`:
    ```bash
    git clone https://bitbucket.org/odedevs/ode.git
    ```
3. `Windows`
   - [Install MinGW-x64](https://code.visualstudio.com/docs/languages/cpp#_example-install-mingwx64)
   - [Install CMake](https://www.msys2.org/docs/cmake/)
   - build `ode`
      ```bash
      cd /xxxpath/ode/build
      cmake ..
      cmake --build .
      ```
    - copy `xxx.dll` and `xxx.a` into `/xxxpath/ode-go/bin`
    - build `ode-go/example`
      ```bash
      cd /xxxpath/ode-go
      go build -o="./bin/plane2d.exe" ./example/plane2d
      go build -o="./bin/chain.exe" ./example/chain
      ```