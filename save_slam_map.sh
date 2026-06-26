#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
MAP_DIR="$HOME/maps/home_map_final"
BAG_OUTPUT="$MAP_DIR"
BAG_PATTERN="$MAP_DIR/*.mcap"
PGM_PATH="$HOME/maps/home_map.pgm"
YAML_PATH="$HOME/maps/home_map.yaml"

cleanup() {
    echo
    echo "SIGINT received: stopping teleop, publishing stop, finalizing bag, and extracting the map..."

    # Avoid re-entrancy
    if [[ "$CLEANING" -ne 0 ]]; then
        echo "Cleanup already running, skipping"
        return
    fi
    CLEANING=1

    # Ensure ROS env available for publishing stop cmd
    source "$ROS_SETUP" || true
    source "$WS_SETUP" || true

    # Publish a zero velocity to stop motors (best-effort) but don't block forever
    echo "Publishing zero /cmd_vel to stop motors (timeout 5s)..."
    timeout 5 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true

    # allow robot to come to rest
    sleep 2

    # Gracefully stop the bag recorder first so MCAP finalizes
    if [[ -n "${BAG_PID:-}" ]]; then
        echo "Sending SIGINT to ros2 bag (pid $BAG_PID) to finalize..."
        kill -INT "$BAG_PID" 2>/dev/null || kill "$BAG_PID" 2>/dev/null || true
        # wait up to 10s for bag to exit
        for i in {1..10}; do
            if kill -0 "$BAG_PID" 2>/dev/null; then
                sleep 1
            else
                break
            fi
        done
    fi

    # Stop slam and bringup
    if [[ -n "${SLAM_PID:-}" ]]; then
        kill "$SLAM_PID" 2>/dev/null || true
    fi
    if [[ -n "${BRINGUP_PID:-}" ]]; then
        kill "$BRINGUP_PID" 2>/dev/null || true
    fi
    wait 2>/dev/null || true

    bag=$(ls -1 ${BAG_PATTERN} 2>/dev/null | sort | tail -n 1 || true)
    if [[ -z "$bag" ]]; then
        echo "ERROR: no bag file found in $MAP_DIR"
        exit 1
    fi

    echo "Attempting Python MCAP extraction from $bag"
    if python3 -c "from pathlib import Path; import yaml; from PIL import Image; from mcap_ros2.reader import read_ros2_messages; bag=Path(r'$bag'); msgs=list(read_ros2_messages(str(bag), '/map')); assert msgs, 'no /map messages'; g=msgs[-1].ros_msg; data=bytes(254 if v==0 else 0 if v==100 else 205 for v in g.data); Image.frombytes('L',(g.info.width,g.info.height),data).save(Path(r'$PGM_PATH')); yaml.safe_dump({'image':'home_map.pgm','resolution':g.info.resolution,'origin':[g.info.origin.position.x,g.info.origin.position.y,0.0],'negate':0,'occupied_thresh':0.65,'free_thresh':0.196,'mode':'trinary'}, open(Path(r'$YAML_PATH'),'w'))"; then
        echo "Python extraction succeeded"
        echo "Saved $PGM_PATH and $YAML_PATH"
        exit 0
    else
        echo "Python extraction failed; attempting fallback via ros2 bag play + map_saver_cli"
        PLAY_PID=""
        ros2 bag play "$bag" --clock &
        PLAY_PID=$!
        sleep 2
        echo "Running map_saver_cli fallback to write maps to $HOME/maps/home_map"
        ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/home_map" || true
        # stop play
        if [[ -n "$PLAY_PID" ]]; then
            kill "$PLAY_PID" 2>/dev/null || true
        fi
        if [[ -f "$HOME/maps/home_map.pgm" && -f "$HOME/maps/home_map.yaml" ]]; then
            echo "Fallback map_saver_cli succeeded: $HOME/maps/home_map.pgm $HOME/maps/home_map.yaml"
            exit 0
        else
            echo "Fallback failed; please inspect the bag file at: $bag"
            exit 1
        fi
    fi
}

trap cleanup INT TERM

source "$ROS_SETUP"
source "$WS_SETUP"
mkdir -p "$MAP_DIR"

echo "Starting robot bringup..."
ros2 launch robot_driver robot_bringup.launch.py &
BRINGUP_PID=$!
sleep 5

echo "Starting slam_toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$HOME/slam_config.yaml" &
SLAM_PID=$!
sleep 5

echo "Starting /map recording..."
ros2 bag record /map -o "$BAG_OUTPUT" &
BAG_PID=$!

echo
echo "Drive forward 5 meters, then rotate 360 degrees slowly. Press Ctrl+C when done to save the map."
echo "Press z to decrease speed, q to increase. Stay at 0.5 speed for safe mapping."

# Start teleop with initial safe speeds (speed=0.5, turn=1.0)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.5 -p turn:=1.0

# When teleop exits (or user presses Ctrl+C), the trap/cleanup will run
wait
