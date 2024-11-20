# TODO:
---------
- go to a previous version, save basic version of click where everything is in one file
- write test for every algorithm - just run it on sample data on simulation and pinocchio.
  using things on the real robot needs to be tested manually, but simulation in URsimulator should
  take care of that because the only difference is the IP address in that case.
  point is test what you can.
- write out types in various functions
- write appropriate asserts over applicable arguments - you don't want to blow something up
  due to a misclick while typing out arguments
- try to get reading and setting the payload from code. also figure out why doesn't it zero the f/t sensor
  --> works, but you need to change ur_rtde code, recompile, and install both c++ and python from there.
so not terribly convenient. it will get fixed in a future ur_rtde release tho
- merge existing visualization with current solution. no, it can't real real-time with the robot
  because it really is that bad, but if you run it in a separate process and update it sporadicly,
  it will be ok. it already has a bunch of the goodies you want so it's certainly easier to get
  this to work than to re-do the same functionality from scratch. 
  and you won't really be using it too much anyway let's be real. AND ALSO,
  if you ditch your shitty integration for pinocchio's it will be faster.
- use logging instead of a debug flag (you're reinventing the wheel and it's also ugly)
- add types to everything, check them with mypy

# installation
------------
first you MUST update pip, otherwise it won't work:
python3 -m pip install --upgrade pip setuptools wheel build
then install with 
pip install --user -e . 
from this directory. now the package is editable, which there's a chance you'd want

# description
---------
- organized as a library called ur_simple_control, made into a python package. the hope is that if this
  is done well enough, you could mix-and-match different components
  and just have them work as intended. on the other hand,
  the code should still be simple enough to afford the quickest possible prototyping,
  which is the point of having it all in python anyway
- initial python solution is age-old code which needs to be remapped into the 
  libraries used here. it will sit here until everything it offers has been translated.

# runnable things
---------------
are in the examples folder
