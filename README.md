# MuNES: Multifloor Navigation Including Elevators and Stairs
Video [[youtube]](https://youtu.be/9saqN6IrsLM)
## Download datasets
[Data 1](https://bit.ly/munes_data_link_1) (moving between two floors; stairs and an elevator)

## Mapping
### Prerequisite
* Our code depend on ROS, Ceres, GTSAM. Before you install MuNES, please install those prerequisites. 
### Execute
```
mkdir -p ~/munes_ws/src
cd ~/munes_ws/src
git clone git@github.com:donghwijung/MuNES.git
cd ../
catkin_make
source ~/munes_ws/devel/setup.bash
roslaunch munes run.launch
rosbag play data_1.bag 
```
## Trajectory Planning
### Prerequisite
* To proceed the trajectory planning, a pcd file containing ground points from the multi-floor map.

* The ground map can be obtained from the mapping process.

* If you want to skip the mapping process, you can download [pcd file](https://bit.ly/munes_ground_map) and move the downloaded file to the directory */pcd* in this repository.
### Setup
* We tested MuNES with *python 3.8*.
```
pip install -r requirements.txt
```
### Execute
#### Planning using the real map (a prior map is necessary)
```
cd planning
python munes.py
```
#### Planning using the virtual map (a prior map is not necessary)
```
cd planning
python munes_wo_map.py
```
## Acknowledgement
The codes in this repository is based on [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM), and [PathPlanning](https://github.com/zhm-real/PathPlanning). Thanks to the authors of those codes.
## Citation
This code is for the paper below:
Donghwi Jung, Jae-Kyung Cho, Younghwa Jung, Soohyun Shin, Seong-Woo Kim, Autonomous Robot Intelligence Lab, Seoul National University.