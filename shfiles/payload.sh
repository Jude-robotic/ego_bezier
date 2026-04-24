
roslaunch ego_planner swarm_payload.launch & sleep 20;

rostopic pub -1 /swarm/formation_mode std_msgs/String "data: 'program_ring'" & sleep 10;

rostopic pub -1 /swarm/formation_mode std_msgs/String "data: 'normal'";

wait;
