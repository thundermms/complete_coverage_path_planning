FROM ros:noetic-ros-core

RUN apt-get update && \
    apt-get install -y --no-install-recommends g++ \
    make \
    ros-noetic-diagnostics \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-angles \
    ros-noetic-eigen-conversions \
    ros-noetic-laser-geometry \
    ros-noetic-move-base-msgs \
    ros-noetic-cob-map-accessibility-analysis \
    coinor-* \
    python3-pip \
    git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

WORKDIR /home/coverage_path_planner


COPY . src/complete_coverage_path_planner

RUN /ros_entrypoint.sh catkin_make && sed -i '$isource "/home/coverage_path_planner/devel/setup.bash"' /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD roslaunch ipa_room_exploration room_exploration_action_server.launch --wait
# CMD bash