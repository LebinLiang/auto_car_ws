source install/setup.bash

ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=B8_3 \
    mode:=nav \
    lio:=fastlio \
    localization:=amcl \
    lio_rviz:=False \
    nav_rviz:=True