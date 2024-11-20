## Docker
----------
In the interest of time, the library is shipped as a Docker image.

Installing docker on windows:
1. install wsl with "wsl --install" in powershell administrator, more at [windows docs](https://learn.microsoft.com/en-us/windows/wsl/install)
2. install docker on windows (with wsl option, should be default), more at [docker docs](https://docs.docker.com/desktop/install/windows-install)
3. create the docker accout 
4. open docker desktop, navigate to Settings (by user/account button) -> Resources -> Network -> click on enable host networking
5. open wsl (type wsl in powershell) and navigate to git folder (now in wsl) with "cd /mnt/Users/YOURUSERNAME/PATH_TO_GIT_FOLDER"

Here are the steps to get to your container running once Docker is installed
6. build the image with "docker build -t ur_simple_control ."
7. to run the image FIRST RUN "xhost +" EVERY TIME, THEN RUN  "docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp:/tmp ur_simple_control /bin/zsh"
8. verify installation by running an example with --visualize-manipulator and --real-time-plotting arguments
9. if you want to make persistent changes to the code from the docker, you need to use the -v argument in docker run to share the folder
   with the code between your native OS (host) and the docker container. 
   in particular, use -v [path_to_folder_on_host]:[path_to_that_folder_in_docker_container]:rw .
   notice the :rw at the end. this allows for both the host and the container to modify the files (otherwise they will be read-only).

### Installing additional software on the image
-------------------------------------------------
Install things as you would normally (using apt in the terminal etc).
To get this to be a part of the Docker image, just use
the RUN [...] command in the Dockerfile, where you replace [...] with the
command you used to install what you installed normally.
Don't forget to build the image to apply these changes!
I highly recommend adding software this way, because
it will allow you to share the exact same set-up between your colleagues,
which will help you avoid headaches in the future.

## Native installation (installing ubuntu)
----------------------------------------------
1. Either create a disk partition for Ubuntu on your hard drive, or use an external hard drive. In the first case, you might need to shrink your existing partition. Searching for "how to create a disk partition [your_OS]" or "install ubuntu on [your_OS]" will get you all the information you need. Ideally, back up your data before any of this (you should be doing this in general as it's good practice).
2. Download an Ubuntu 22 LTH iso (file which contains everything needed for installation), available at official Ubuntu websites
3. Create a bootable USB. In short, you can't just copy an iso file to a USB and boot from it. Just google how to do it on your OS (use rufus if on windows).
4. Enter BIOS to ensure you can boot from your USB. Ideally this step should just be to enable Boot menu, but this step is dependent to a specific computer. Any reasonable Ubuntu (or any Linux) installation guide should have detailed steps, but also consult your laptop manufacturer websites for BIOS steps if the steps won't be obvious/easy to follow.
5. Boot from the Ubuntu USB and install it to the partition/external disk from step 1. MAKE SURE you are installing to the right partition. To select the partition, select the "Advanced installation" option (selecting the partition is the only advanced thing you need). Otherwise just follow the installation instructions you get once you boot into the USB, they are quite well done.

