/********************************************************************
* Description: invkins.c
*   Inverse only kinematics for testing
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   and trivkins.c
*
* Author: Charles Steinkuehler
* License: GPL Version 2 or later
* System: Linux
*    
* Copyright (c) 2015 All rights reserved.
*
* Last change:
********************************************************************/

#include "kinematics.h"		/* these decls */


int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    return -1;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    joints[0] = pos->tran.y;
    joints[1] = pos->tran.x + pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_INVERSE_ONLY;
}

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) {
    comp_id = hal_init("invkins");
    if(comp_id > 0) {
	hal_ready(comp_id);
	return 0;
    }
    return comp_id;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
