# MuNES: Multifloor Navigation Including Elevators and Stairs
Video [[link]](https://youtu.be/9saqN6IrsLM) Paper [[link]](https://arxiv.org/pdf/2402.04535.pdf)
## Download datasets
[Data 1](https://mysnu-my.sharepoint.com/:u:/g/personal/donghwijung_seoul_ac_kr/EaACWm8E9uRLv-scGSS4PxUBhdUgzb0sfgV6d7IKrM6SKg?e=A0EWhl) (moving between two floors; stairs and an elevator)

## Mapping
### Prerequisite
* Our code depends on ROS, Ceres, GTSAM, thus please install those prerequisites before installing MuNES. 
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
* To proceed the trajectory planning, a pcd file containing ground points of the multi-floor map is necessary.

* The ground map can be obtained from the mapping process.

* If you want to skip the mapping process, you can download [pcd file](https://bit.ly/munes_ground_map_link) and move the downloaded file to the directory */pcd* in this repository.
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

## Paper
```
@article{jung2024munes,
  title={MuNES: Multifloor Navigation Including Elevators and Stairs},
  author={Jung, Donghwi and Kim, Chan and Cho, Jae-Kyung and Kim, Seong-Woo},
  journal={arXiv preprint arXiv:2402.04535},
  year={2024}
}
```

## Acknowledgement
The code in this repository is based on [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM), and [PathPlanning](https://github.com/zhm-real/PathPlanning). Thanks to the authors of those codes.

## License

Copyright 2024, Donghwi Jung, Chan Kim, Jae-Kyung Cho, and Seong-Woo Kim, Autonomous Robot Intelligence Lab, Seoul National University.

This project is free software made available under the MIT License. For details see the LICENSE file.