# pinocchio
-------------
pinocchio has functions with some advanced math, like Lie groups.  while it's
certainly a plus to learn them, you don't need to. you can just use the forward
kinematics and calculating jacobian functions, and do ther rest of the math
with eigen or numpy.

## general documentation
-------------------------------
[pinocchio documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/index.html)

## native installation procedure
-------------------------------
just follow the steps (copy-paste commands into terminal) listed 
[here](https://stack-of-tasks.github.io/pinocchio/download.html) .
NOTE: if using own compilation of pinocchio, make sure you compile with mpi 
and whatever else is needed to run algorithms in parallel.
you won't be able to run in 500Hz on a single core. also note that
the compilation required 13GB of RAM on my system. 

## tutorials
-------------------------------
- [probably the best introduction to pinocchio](https://github.com/ymontmarin/_tps_robotique)
- [pinocchio with casadi](https://github.com/nmansard/jnrh2023) - use the provided Docker, very difficult to install the exact software version used in the tutorial
- [exercises offered on documentation webstite](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_d-practical-exercises_intro.html)


