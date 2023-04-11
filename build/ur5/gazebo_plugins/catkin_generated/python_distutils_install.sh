#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/divishad/test_ws/src/ur5/gazebo_plugins"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/divishad/test_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/divishad/test_ws/install/lib/python3/dist-packages:/home/divishad/test_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/divishad/test_ws/build" \
    "/usr/bin/python3" \
    "/home/divishad/test_ws/src/ur5/gazebo_plugins/setup.py" \
     \
    build --build-base "/home/divishad/test_ws/build/ur5/gazebo_plugins" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/divishad/test_ws/install" --install-scripts="/home/divishad/test_ws/install/bin"
