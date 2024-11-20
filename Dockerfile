FROM ubuntu:jammy

LABEL org.opencontainers.image.authors="marko.guberina@control.lth.se"

# install python3-tk without questions
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Stockholm

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3  \
        python3-pip \
        python3-tk \
        python3-dev \
        ipython3 \
        git \
        sudo \
        man-db \
        manpages-posix \
        arp-scan \
        # nice to have
        vim \
        vim-addon-manager \
        vim-youcompleteme \
        vim-python-jedi \
        zsh \
        zsh-syntax-highlighting \
        python3-python-qt-binding \
        build-essential \
        gcc \
        v4l-utils
        
        
# qt-binding is a really unnecessary 300MB, but i don't want
# to do more matplotlib hacks

RUN sed -i 's:^path-exclude=/usr/share/man:#path-exclude=/usr/share/man:' \
        /etc/dpkg/dpkg.cfg.d/excludes
RUN yes | unminimize 

# make the environment more usable
# create user
RUN useradd -m -s /bin/zsh -G sudo -u 1000 student

WORKDIR /home/student/
RUN passwd -d student
USER student
# copy repo to workdir
COPY --chown=student . .
RUN mkdir -p .cache/zsh/
COPY --chown=student /dot_files_for_docker/.vimrc /home/student/
COPY --chown=student /dot_files_for_docker/.zshrc /home/student/
COPY --chown=student /dot_files_for_docker/global_extra_conf.py /home/student/

RUN vam install python-jedi && vam install youcompleteme

# this is enough to run clik
WORKDIR /home/student/
RUN pip install -e ./python/
RUN pip install roboticstoolbox-python kociemba \
                opencv-python torch pandas
RUN pip install pin matplotlib meshcat ur_rtde \
                qpsolvers ecos                