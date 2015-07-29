This is a copy of ODE 0.9-rc1.

ODE apparently does not support using a different collision detection library
directly, without modification of the ODE source, see
 http://ode.org/pipermail/ode/2007-September/022650.html
 http://ode.org/pipermail/ode/2007-October/022699.html
 http://ode.org/pipermail/ode/2007-October/022700.html

ODE is licensed under the modified (i.e. 3 clause) BSD license, see
LICENSE-BSD.TXT. So is this copy of ODE (along with the modifications of the
source).


The d-collide interface to ODE is defined in
  include/ode/odewrapper.h
See also our sample applications (the "testapp" directory")
