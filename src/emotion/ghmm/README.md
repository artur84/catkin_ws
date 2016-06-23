Compiling this module
---------------------

This module has the following dependencies:

* boost-graph (libboost-graph-dev in Ubuntu and Debian).
* ghmm (at github http://github.com/dichodaemon/ghmm, more on this below).

GHMM
----

GHMM is template based, so it's header only. You can download it from
github. For debian-like systems, you can also generate a debian package, 
the procedure is more or less as follows:

    git clone https://github.com/dichodaemon/ghmm.git
    cd ghmm
    mkdir build
    cd build
    cmake .. -DDEBIAN_ARCHITECTURE=all
    make deb
    dpkg -i ghmm-dev*.deb

