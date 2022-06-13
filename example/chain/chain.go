package main

/*
#cgo CPPFLAGS: -I../../../ode/include
#cgo LDFLAGS: -L${SRCDIR}/../../bin/ -l:libode.a -l:libdrawstuff.a
#include <drawstuff/drawstuff.h>
#include "chain_capi.h"
*/
import "C"

import (
	"flag"
	"math"
	"ode"
	"unsafe"
)

const (
	numSpheres   = 10
	sideLen      = 0.2
	sphereMass   = 1
	sphereRadius = 0.1732
)

var (
	world  ode.World
	space  ode.Space
	body   []ode.Body
	joint  []ode.BallJoint
	ctGrp  ode.JointGroup
	sphere []ode.Sphere
	mass   *ode.Mass
	angle  float64
)

func cbNear(_ interface{}, obj1, obj2 ode.Geom) {
	contact := ode.NewContact()
	body1, body2 := obj1.Body(), obj2.Body()
	if body1 != 0 && body2 != 0 && body1.Connected(body2) {
		return
	}
	contact.Surface.Mode = 0
	contact.Surface.Mu = 0.1
	contact.Surface.Mu2 = 0
	cts := obj1.Collide(obj2, 1, 0)
	if len(cts) > 0 {
		contact.Geom = cts[0]
		ct := world.NewContactJoint(ctGrp, contact)
		ct.Attach(body1, body2)
	}
}

var texturePath string

func main() {
	flag.StringVar(&texturePath, "texture", "../textures", "textures path")
	flag.Parse()
	ode.Init(0, ode.AllAFlag)

	world = ode.NewWorld()
	space = ode.NilSpace().NewHashSpace()
	body = make([]ode.Body, numSpheres)
	joint = make([]ode.BallJoint, numSpheres-1)
	ctGrp = ode.NewJointGroup(1000000)
	sphere = make([]ode.Sphere, numSpheres)
	mass = ode.NewMass()

	world.SetGravity(ode.V3(0, 0, -0.5))
	space.NewPlane(ode.V4(0, 0, 1, 0))

	for i := 0; i < numSpheres; i++ {
		k := float64(i) * sideLen
		body[i] = world.NewBody()
		body[i].SetPosition(ode.V3(k, k, k+0.4))
		mass.SetBox(1, ode.V3(sideLen, sideLen, sideLen))
		mass.Adjust(sphereMass)
		body[i].SetMass(mass)
		sphere[i] = space.NewSphere(sphereRadius)
		sphere[i].SetBody(body[i])
	}

	for i := 0; i < numSpheres-1; i++ {
		joint[i] = world.NewBallJoint(ode.JointGroup(0))
		joint[i].Attach(body[i], body[i+1])
		k := (float64(i) + 0.5) * sideLen
		joint[i].SetAnchor(ode.V3(k, k, k+0.4))
	}

	drawstuff_functions := &C.struct_dsFunctions{
		version:          0x0002,
		start:            (*[0]byte)(unsafe.Pointer(C.cb_start)),
		step:             (*[0]byte)(unsafe.Pointer(C.cb_step)),
		command:          nil,
		stop:             nil,
		path_to_textures: C.CString(texturePath),
	}
	C.dsSimulationLoop(0, (**C.char)(nil), 800, 600, drawstuff_functions)
}

//export cb_sim_start
func cb_sim_start() {
	xyz := []float32{-2, 0, 15 * sideLen}
	hpr := []float32{0, -45.0, 0}
	C.dsSetViewpoint((*C.float)(&xyz[0]), (*C.float)(&hpr[0]))
}

//export cb_sim_step
func cb_sim_step(pause C.int) {
	if pause == 0 {
		angle += 0.05
		body[len(body)-1].AddForce(ode.V3(0, 0, 1.5*(math.Sin(angle)+1)))
		space.Collide(0, cbNear)
		world.Step(0.05)
		ctGrp.Empty()
	}

	C.dsSetTexture(C.DS_WOOD)
	for b := 0; b < numSpheres; b++ {
		if b == 0 {
			C.dsSetColor(1.0, 0.5, 1.0)
		} else {
			C.dsSetColor(0, 0.5, 1.0)
		}
		pos := body[b].Position()
		rot := body[b].Rotation()
		C.dsDrawSphereD((*C.double)(&pos[0]), (*C.double)(&rot[0][0]), C.float(sphereRadius))
	}
}
