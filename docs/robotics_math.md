## general notes
---------------------
You do not need to know most topics in the books listed below, but it's good
to have them as references and to look up the specific topics you are
working with. We will rely on the Pinocchio library for robotics-related
calculations like forward kinematics, and on Eigen or NumPy for
general linear algebra operations. You really only need to 
know homogenious transformations, and have a good idea of
what the used equations mean (what are you calculating
and from what).

## classic approach
-----------------------
probably the best reference textbook:
Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, Giuseppe Oriolo (auth.) - Robotics_ Modelling, Planning and Control.
the only prerequisites are linear algebra, calculus and basic control (up to SISO PID control).
If you took the Applied robotics course, this book builds on what has been laid out there.
While easy to get started with, this approach may result in some cumbersome expressions.

## Lie group approach
-----------------------------
The idea here is to treat rigid motions and transformations as (mathematical) groups.
While it is certainly not necessary to use this approach,
it sheds light on the expressions which appear in robotics, and results
in a natural and intuitive framework.
A modern, excellent take: Frank C. Park Kevin M. Lynch - Modern Robotics Mechanics, Planning, and Control.
Everything is laid out and explained lucidly.
A more technical take can be found in:
A mathematical introduction to robotic manipulation - R. Murray, Z. Li, S. Sastry.
While an excellent read for those who want to go deeper into the theory, it will
most likely take a while to go through. 

