// AB: code from ODE, which has been replaced by our own implementations.
//     we keep the ODE code here to provide the possibility to switch between ODE
//     and d-collide collision detection

/******** from collision_space.cpp *********/
#include <ode/common.h>
#include <ode/collision_space.h>
#include <ode/collision.h>
#include "collision_kernel.h"
#include "collision_space_internal.h"
/******** from collision_space.cpp (end) *********/

void dChangeCollisionDetection(void (*_dGeomMoved)(dGeomID),
              dGeomID (*_dGeomGetBodyNext)(dGeomID),
              void (*_dGeomSetBody)(dGeomID, dBodyID),
              dBodyID (*_dGeomGetBody)(dGeomID)) {
    dGeomMoved = _dGeomMoved;
    dGeomGetBodyNext = _dGeomGetBodyNext;
    dGeomSetBody = _dGeomSetBody;
    dGeomGetBody = _dGeomGetBody;
}

/******** from collision_kernel.cpp *********/
dxPosR* dAllocPosr();
void dFreePosr(dxPosR*);
/******** from collision_kernel.cpp (end) *********/


// AB: from collision_space.cpp
//****************************************************************************
// make the geom dirty by setting the GEOM_DIRTY and GEOM_BAD_AABB flags
// and moving it to the front of the space's list. all the parents of a
// dirty geom also become dirty.

void dGeomMovedODE (dxGeom *geom)
{
  dAASSERT (geom);

  // if geom is offset, mark it as needing a calculate
  if (geom->offset_posr) {
    geom->gflags |= GEOM_POSR_BAD;
  }

  // from the bottom of the space heirarchy up, process all clean geoms
  // turning them into dirty geoms.
  dxSpace *parent = geom->parent_space;

  while (parent && (geom->gflags & GEOM_DIRTY)==0) {
    CHECK_NOT_LOCKED (parent);
    geom->gflags |= GEOM_DIRTY | GEOM_AABB_BAD;
    parent->dirty (geom);
    geom = parent;
    parent = parent->parent_space;
  }

  // all the remaining dirty geoms must have their AABB_BAD flags set, to
  // ensure that their AABBs get recomputed
  while (geom) {
    geom->gflags |= GEOM_DIRTY | GEOM_AABB_BAD;
    CHECK_NOT_LOCKED (geom->parent_space);
    geom = geom->parent_space;
  }
}
void (*dGeomMoved)(dGeomID) = &dGeomMovedODE;


// AB: from collision_kernel.cpp

dxGeom *dGeomGetBodyNextODE (dxGeom *geom)
{
  return geom->body_next;
}
dGeomID (*dGeomGetBodyNext)(dGeomID) = &dGeomGetBodyNextODE;

void dGeomSetBodyODE (dxGeom *g, dxBody *b)
{
  dAASSERT (g);
  dUASSERT (b == NULL || (g->gflags & GEOM_PLACEABLE),"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);

  if (b) {
    if (!g->body) dFreePosr(g->final_posr);
    if (g->body != b) {
      if (g->offset_posr) {
        dFreePosr(g->offset_posr);
        g->offset_posr = 0;
      }
      g->final_posr = &b->posr;
      g->bodyRemove();
      g->bodyAdd (b);
    }
    dGeomMoved (g);
  }
  else {
    if (g->body) {
      if (g->offset_posr)
      {
        // if we're offset, we already have our own final position, make sure its updated
        g->recomputePosr();
        dFreePosr(g->offset_posr);
        g->offset_posr = 0;
      }
      else
      {
        g->final_posr = dAllocPosr();
        memcpy (g->final_posr->pos,g->body->posr.pos,sizeof(dVector3));
        memcpy (g->final_posr->R,g->body->posr.R,sizeof(dMatrix3));
      }
      g->bodyRemove();
    }
    // dGeomMoved() should not be called if the body is being set to 0, as the
    // new position of the geom is set to the old position of the body, so the
    // effective position of the geom remains unchanged.
  }
}
void (*dGeomSetBody)(dGeomID, dBodyID) = &dGeomSetBodyODE;


dBodyID dGeomGetBodyODE (dGeomID g)
{
  dAASSERT (g);
  return g->body;
}
dBodyID (*dGeomGetBody)(dGeomID) = &dGeomGetBodyODE;

