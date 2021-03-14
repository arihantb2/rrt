BUILD_DIRECTORY="build"
BUILD_STATUS_FILE=".build_incomplete"
RUN_FILE="build/rrt"
CONFIG_FILE="config/rrt_config.yaml"

if ! test -d "$BUILD_DIRECTORY"; then
    echo "Build not executed or incomplete. Building application now.."
    ./build.sh
fi

if test -f "$BUILD_STATUS_FILE"; then
    echo "Build not executed or incomplete. Building application now.."
    ./build.sh
fi

if ! test -f "$RUN_FILE"; then
    echo "Build not executed or incomplete. Building application now.."
    ./build.sh
fi

chmod +x $RUN_FILE
./$RUN_FILE $CONFIG_FILE
