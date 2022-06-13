#include <ode/ode.h>
#include <stdio.h>

extern void nearCallback(void* data, dGeomID obj1, dGeomID obj2);
extern void movedCallback(dBodyID body);
extern int triCallback(dGeomID mesh, dGeomID other, int index);
extern int triRayCallback(dGeomID mesh, dGeomID ray, int index, dReal u, dReal v);

void callNearCallback(void* data, dGeomID obj1, dGeomID obj2) {
  nearCallback(data, obj1, obj2);
}

void callMovedCallback(dBodyID body) {
  movedCallback(body);
}

int callTriCallback(dGeomID mesh, dGeomID other, int index) {
  return triCallback(mesh, other, index);
}

int callTriRayCallback(dGeomID mesh, dGeomID ray, int index, dReal u, dReal v) {
  return triRayCallback(mesh, ray, index, u, v);
}
