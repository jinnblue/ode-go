package main

/*
#cgo CPPFLAGS: -I../../../ode/include
#cgo LDFLAGS: -L${SRCDIR}/../../bin/ -l:libode.a -l:libdrawstuff.a
#include <drawstuff/drawstuff.h>
#include "plane_capi.h"
*/
import "C"

import (
	"flag"
	"math"
	"math/rand"
	"ode"
	"unsafe"
)

const (
	N_BODIES   = 40
	STAGE_SIZE = 8.0 // in m
	TIME_STEP  = 0.01
)

var (
	dyn_world         ode.World
	dyn_bodies        []ode.Body
	bodies_sides      []ode.Vector3
	coll_space        ode.SimpleSpace
	plane2d_joint_ids []ode.Plane2DJoint
	coll_contacts     ode.JointGroup
)

var texturePath string

func main() {
	flag.StringVar(&texturePath, "texture", "../textures", "textures path")
	flag.Parse()

	ode.Init(0, ode.AllAFlag)

	dyn_world = ode.NewWorld()
	dyn_world.SetERP(0.5)
	dyn_world.SetCFM(0.001)
	dyn_world.SetGravity(ode.V3(0, 0.0, -1.0))
	coll_space = ode.NilSpace().NewSimpleSpace()
	dyn_bodies = make([]ode.Body, N_BODIES)
	bodies_sides = make([]ode.Vector3, N_BODIES)
	plane2d_joint_ids = make([]ode.Plane2DJoint, N_BODIES)
	coll_contacts = ode.NewJointGroup(1000000)
	mass := ode.NewMass()

	for b := 0; b < N_BODIES; b++ {
		l := int(1 + math.Sqrt(N_BODIES))
		x := (0.5 + float64(b/l)) / float64(l) * STAGE_SIZE
		y := (0.5 + float64(b%l)) / float64(l) * STAGE_SIZE
		z := 1.0 + 0.1*rand48()
		bodies_sides[b] = ode.V3(
			5*(0.2+0.7*rand48())/math.Sqrt(N_BODIES),
			5*(0.2+0.7*rand48())/math.Sqrt(N_BODIES),
			z,
		)

		dyn_bodies[b] = dyn_world.NewBody()
		dyn_bodies[b].SetPosition(ode.V3(x, y, z/2))
		dyn_bodies[b].SetData(b)
		dyn_bodies[b].SetLinearVelocity(ode.V3(3*rand48()-0.5, 3*rand48()-0.5, 0))

		mass.SetBox(1, bodies_sides[b])
		mass.Adjust(0.1 * bodies_sides[b][0] * bodies_sides[b][1])
		dyn_bodies[b].SetMass(mass)

		plane2d_joint_ids[b] = dyn_world.NewPlane2DJoint(ode.JointGroup(0))
		plane2d_joint_ids[b].Attach(dyn_bodies[b], ode.Body(0))
	}

	plane2d_joint_ids[0].SetXParam(ode.FMaxJtParam, 10)
	plane2d_joint_ids[0].SetYParam(ode.FMaxJtParam, 10)

	// collision geoms and joints
	coll_space.NewPlane(ode.V4(1, 0, 0, 0))
	coll_space.NewPlane(ode.V4(-1, 0, 0, -STAGE_SIZE))
	coll_space.NewPlane(ode.V4(0, 1, 0, 0))
	coll_space.NewPlane(ode.V4(0, -1, 0, -STAGE_SIZE))

	for b := 0; b < N_BODIES; b++ {
		box := coll_space.NewBox(bodies_sides[b])
		box.SetBody(dyn_bodies[b])
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

func rand48() float64 {
	return rand.Float64()
}

var angle float64

func track_to_pos(body ode.Body, joint2d ode.Plane2DJoint, targetX float64, targetY float64) {
	pos := body.Position()
	currX := pos[0]
	currY := pos[1]

	joint2d.SetXParam(ode.VelJtParam, 1*(targetX-currX))
	joint2d.SetYParam(ode.VelJtParam, 1*(targetY-currY))
}

//export cb_sim_start
func cb_sim_start() {
	xyz := []float32{0.5 * STAGE_SIZE, 0.5 * STAGE_SIZE, 0.65 * STAGE_SIZE}
	hpr := []float32{90.0, -90.0, 0}
	C.dsSetViewpoint((*C.float)(&xyz[0]), (*C.float)(&hpr[0]))
}

//export cb_sim_step
func cb_sim_step(pause C.int) {
	if pause == 0 {
		angle += 0.01
		track_to_pos(dyn_bodies[0], plane2d_joint_ids[0],
			STAGE_SIZE/2+STAGE_SIZE/2.0*math.Cos(angle),
			STAGE_SIZE/2+STAGE_SIZE/2.0*math.Sin(angle),
		)

		const n = 10
		for i := 0; i < n; i++ {
			coll_space.Collide(nil, cbNear)
			dyn_world.Step(TIME_STEP / n)
			coll_contacts.Empty()
		}
	}

	if true {
		// @@@ hack Plane2D constraint error reduction here:
		for b := 0; b < N_BODIES; b++ {
			rot := dyn_bodies[b].AngularVel()
			quat := dyn_bodies[b].Quaternion()
			quat_len := math.Sqrt(quat[0]*quat[0] + quat[3]*quat[3])
			quat[0] /= quat_len
			quat[1] = 0
			quat[2] = 0
			quat[3] /= quat_len
			dyn_bodies[b].SetQuaternion(quat)
			rot[0] = 0
			rot[1] = 0
			dyn_bodies[b].SetAngularVelocity(rot)
		}
	}

	if false {
		const s = 1.00
		const t = 0.99
		for b := 0; b < N_BODIES; b++ {
			vel := dyn_bodies[b].LinearVelocity()
			rot := dyn_bodies[b].AngularVel()
			vel[0] *= s
			vel[1] *= s
			vel[2] *= s
			dyn_bodies[b].SetLinearVelocity(vel)
			rot[0] *= t
			rot[1] *= t
			rot[2] *= t
			dyn_bodies[b].SetAngularVelocity(rot)
		}
	}

	C.dsSetTexture(C.DS_WOOD)
	for b := 0; b < N_BODIES; b++ {
		if b == 0 {
			C.dsSetColor(1.0, 0.5, 1.0)
		} else {
			C.dsSetColor(0, 0.5, 1.0)
		}
		pos := dyn_bodies[b].Position()
		rot := dyn_bodies[b].Rotation()
		side := bodies_sides[b]
		C.dsDrawBoxD((*C.double)(&pos[0]), (*C.double)(&rot[0][0]), (*C.double)(&side[0]))
		// C.dsDrawBox((*C.double)(unsafe.Pointer(&pos[0])), (*C.double)(unsafe.Pointer(&rot[0])), (*C.double)(unsafe.Pointer(&side[0])))
	}
}

func cbNear(_ interface{}, obj1, obj2 ode.Geom) {
	body1, body2 := obj1.Body(), obj2.Body()

	// exit without doing anything if the two bodies are static
	if (body1 == 0) && (body2 == 0) {
		return
	}

	// exit without doing anything if the two bodies are connected by a joint
	if (body1 != 0) && (body2 != 0) && body1.Connected(body2) {
		/* MTRAP; */
		return
	}

	contact := ode.NewContact()
	contact.Surface.Mode = 0
	contact.Surface.Mu = 0 // frictionless
	cts := obj1.Collide(obj2, 1, 0)
	if len(cts) > 0 {
		contact.Geom = cts[0]
		ct := dyn_world.NewContactJoint(coll_contacts, contact)
		ct.Attach(body1, body2)
	}
}
