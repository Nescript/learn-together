---
tags:
  - type/snippet
  - topic/robomaster
  - topic/shell
  - status/done
date: 2026-01-10
---
```bash
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
        # We have color support; assume it's compliant with Ecma-48
        # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
        # a case would tend to support setf rather than setaf.)
        color_prompt=yes
    else
        color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

export ROBOT_TYPE=standard6
source /opt/ros/noetic/setup.bash
source /home/dynamicx/rm_ws/devel/setup.bash
#source /opt/intel/openvino_2021/bin/setupvars.sh
#source /opt/intel/openvino_2022/setupvars.sh
source /opt/intel/openvino_2024/setupvars.sh

alias setcan0="sudo ip link set can0 up type can bitrate 1000000"
alias setcan1="sudo ip link set can1 up type can bitrate 1000000"
alias clean_ros_log="rm -r ~/.ros/log/"
alias openbash="vim ~/.bashrc"
alias initmit='bash ~/script/bash/init_mit_motor.sh'
alias closemit='bash ~/script/bash/close_mit_motor.sh'

export ROS_IP=`ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep -v '172.17.0.1'`
if  test -z "${ROS_IP}"
then
       export ROS_IP=`dig +short localhost`
fi

export ROS_IP=192.168.100.2
#export ROS_IP=192.168.31.207
#export ROS_IP=192.168.1.116
export ROS_MASTER_URI=http://${ROS_IP}:11311

export HW_NAME=rm_ecat_hw
export IMU_TRIGGER=false

export CAMERA_TYPE=hk_camera
export CAMERA_CLASS=HKCameraNodelet




alias stop="sudo systemctl stop rm_ecat_start.service start_master.service"
alias stoprm="sudo systemctl stop rm_start.service"
alias stoprme="sudo systemctl stop rm_ecat_start.service"
alias stopst="sudo systemctl stop start_master.service"
alias stopvi="sudo systemctl stop vision_start.service"
alias startrm="sudo systemctl start rm_start.service"
alias startrme="sudo systemctl start rm_ecat_start.service"
alias startst="sudo systemctl start start_master.service"
alias startvi="sudo systemctl start vision_start.service"
alias statusrm="sudo systemctl status rm_start.service"
alias statusrme="sudo systemctl status rm_ecat_start.service"
alias restart="sudo systemctl restart rm_ecat_start.service start_master.service vision_start.service"
alias statusst="sudo systemctl status start_master.service"
alias restartrm="sudo systemctl restart rm_start.service"
alias restartrme="sudo systemctl restart rm_ecat_start.service"
alias restartst="sudo systemctl restart start_master.service"
alias restartvi="sudo systemctl restart vision_start.service"
alias redbigbuff="rosservice call /Processor/status_switch 0 1 0 0 0 && rosservice call /exposure_status_switch 0 1 0 0 0 && rosservice call /buff_status_switch 0 1 0 0 0 && rosservice call /forecast/status_switch 0 1 0 0 0"
alias bluebigbuff="rosservice call /Processor/status_switch 0 1 0 0 0 && rosservice call /exposure_status_switch 1 1 0 0 0 && rosservice call /buff_status_switch 1 1 0 0 0 && rosservice call /forecast/status_switch 0 1 0 0 0"

alias buff="rosservice call /Processor/status_switch 0 1 0 0 0 && rosservice call /exposure_status_switch 0 1 0 0 0 && rosservice call /buff_status_switch 0 1 0 0 0"






export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol

export ALLUSERSPROFILE=/opt/MVS/MVFG


#export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

export MVCAM_SDK_PATH=/opt/MVS

export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
```