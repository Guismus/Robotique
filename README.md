# Tek5 — SO-ARM101 pick & place (phase 1)

An SO-ARM101 arm driven by ROS 2 (Jazzy) inside Docker, in MuJoCo simulation or on the real arm.
A fixed camera above the table finds the red cube, the brain computes the joint angles (IK), picks the
cube and drops it into the box, then checks every step with the camera (closed loop).

| Branch | Content |
|---|---|
| **`main`** (this one) | **Phase 1**: cube → box pick & place |
| `bonus` | **Phase 2**: 6-cube pyramid with a PyTorch vision model (see `bonus/README.md` there) |
| `pile-3-cubes-vrai-bras` | Phase 1 + a simple 3-cube stack (`MODE=stack`), meant for tests on the real arm |

Project spec: [`docs/SUJET.md`](docs/SUJET.md). Step-by-step guide (French): [`docs/GUIDE_DEBUTANT.md`](docs/GUIDE_DEBUTANT.md).
Report on the first version (French): [`docs/COMPTE_RENDU.md`](docs/COMPTE_RENDU.md).

## Results (simulation)

| What | How | Result |
|---|---|---|
| Success rate | 20 random scenes (seeds 1 to 20), `ros2_ws/checks/taux_reussite.sh` | **20 / 20** (19 at the first attempt, 1 at the second) — `ros2_ws/checks/taux_reussite.txt` |
| Camera calibration (ArUco) | 4 markers, 16 corners, `solvePnP` | 2.1 mm from the true camera pose, 0.56 px reprojection error |
| Rates | `ros2 topic hz` | camera and overlay 10 Hz, `/joint_states` ~23 Hz (30 Hz target, CPU busy with rendering) |

## Layout

```
docker/                       containers: control (driver + brain), perception, viz (robot TF)
  docker-compose.yml          SIM_SEED=N (random scene), USE_SIM=true|false
  control_entrypoint.sh       colcon build, then starts driver + brain
sim/so101_sim/                MuJoCo simulation (same interface as the real arm)
  assets/so101/scene_tek5.xml   1 cube, the box, 4 ArUco markers on the table
ros2_ws/src/
  so101_driver/               only node talking to the arm (sim or real): /joint_command -> /joint_states, camera
  so101_perception/           camera -> 3D positions of the cube and the box, ArUco calibration, overlay, TF
  so101_brain/                FK/IK (kinematics.py) and state machine (brain_node.py)
  so101_description/          URDF + robot_state_publisher (TF of the arm)
ros2_ws/checks/               measurement and debug scripts (success rate, ArUco markers, videos)
ros2_ws/so101.rviz            RViz view: robot, TF frames, camera overlay, detected cube and box, gripper (FK)
```

### Main topics

| Topic | Type | Role |
|---|---|---|
| `/joint_command` → `/joint_states` | JointState | command and measure of the 6 motors |
| `/external_cam/image_raw` | Image | camera, *sensor data* QoS (BEST_EFFORT): a lost image is not sent again, the next one is newer |
| `/object_position`, `/drop_box_position` | PoseStamped | cube and box, in `base_link` |
| `/perception/overlay` | Image | camera image with the detections drawn (cube in green, box in orange, ArUco markers in blue) |
| `/perception/markers` | MarkerArray | cube and box as 3D shapes in RViz |
| `/tf` | TF | frames `camera` (from the ArUco calibration), `cube` and `bac` (box), under `base_link` |
| `/end_effector_pose` | PoseStamped | gripper position computed with the FK |
| `/brain_state` | String | current step (APPROCHE, DESCENTE, SAISIE, ..., CYCLE_REUSSI, ECHEC_...) |
| `/brain/start`, `/brain/stop` | Trigger | start / stop a cycle |

## How it works

1. **Calibration**: at start-up, the 4 ArUco markers stuck on the table (known positions) give the camera
   pose with `solvePnP`. Using 4 markers instead of 1 brings the error from 7–18 mm down to 2 mm.
2. **Perception**: red and brown areas are found in HSV; the centre pixel of each is projected onto a
   known height (cube centre at z = 2.5 cm), which gives its 3D position.
3. **IK**: `kinematics.py` reads the URDF and solves the 5 joint angles by least squares (scipy), with the
   gripper pointing down. The FK of the same chain is published on `/end_effector_pose`.
4. **Brain**: one step at a time (APPROCHE, DESCENTE, SAISIE, LEVAGE, TRANSPORT, DEPOT, RETRAIT),
   smoothed commands (0.025 rad every 50 ms).
5. **Closed loop**: after TRANSPORT the camera checks the cube has left its place (otherwise `SAISIE_RATEE`),
   after RETRAIT that it is in the box (otherwise `PAS_DANS_LE_BAC`). On failure the cycle restarts from
   the new cube position, 3 attempts max. If the arm hides the cube, it first turns aside (DEGAGEMENT) so
   the camera can see the table.

## Run it (Ubuntu / WSL terminal, from `docker/`)

```bash
xhost +local:docker
SIM_SEED=5 docker compose up -d --force-recreate control perception viz
docker compose logs -f perception      # wait for "Calibration ArUco", then Ctrl+C

# RViz (own container, WSLg display)
docker rm -f rviz; docker run -d --name rviz --network tek5_rosnet \
  -e ROS_DOMAIN_ID=42 -e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v "$PWD/../ros2_ws:/ros2_ws" -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  tek5/control bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && rviz2 -d /ros2_ws/so101.rviz"

# Start a cycle and follow the steps
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && ros2 service call /brain/start std_srvs/srv/Trigger"
docker compose logs -f control | grep brain
```

Measure the success rate (restarts the simulation for each seed): `bash ../ros2_ws/checks/taux_reussite.sh 1 20`.

## Real arm

In `docker/.env`: `USE_SIM=false`, `REAL_ROBOT_PORT=/dev/ttyACM0`, `CALIBRATION_FILE=<lerobot file>`.
A camera publishing `/external_cam/image_raw` and `/external_cam/camera_info` is also needed, with the 4 ArUco
markers printed (10 cm black squares, positions in `perception_node.py`, images made by
`ros2_ws/checks/make_aruco.py`).

## Known limits

- The cube orientation (rotation around z) is not used: the gripper always takes it along the same axis.
- HSV colour thresholds are tuned on the simulation; they must be tuned again under real lighting.
- Not yet tested on the real arm.
