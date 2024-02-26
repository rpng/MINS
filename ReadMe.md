# MINS
[![Docker Image CI](https://github.com/rpng/MINS/actions/workflows/docker-image.yml/badge.svg)](https://github.com/rpng/MINS/actions/workflows/docker-image.yml)

An efficient, robust, and tightly-coupled **Multisensor-aided Inertial Navigation System (MINS)** which is capable of 
flexibly fusing all five sensing modalities (**IMU**, **wheel** **encoders**, **camera**, **GNSS**, and **LiDAR**) in a filtering 
fashion by overcoming the hurdles of computational complexity, sensor asynchronicity, and intra-sensor calibration. 

Exemplary use case of MINS: 
* VINS (mono, stereo, multi-cam)
* GPS-IMU (single, multiple)
* LiDAR-IMU (single, multiple)
* wheel-IMU
* Camera-GPS-LiDAR-wheel-IMU or more combinations.

![alt text](thirdparty/frames.png)
![alt text](thirdparty/kaist38.gif)

* Publication reference - [https://arxiv.org/pdf/2309.15390.pdf](https://arxiv.org/pdf/2309.15390.pdf)
  
## Key Features
* Inertial(IMU)-based multi-sensor fusion including wheel odometry and arbitrary numbers of cameras, LiDARs, and GNSSs (+ VICON or loop-closure) for localization.
* Online calibration of all onboard sensors (check [exemplary results](https://github.com/rpng/mins/blob/master/mins_eval/ReadMe.md#run-example)).
* Consistent high-order state on manifold interpolation improved from our prior work ([MIMC-VINS](https://ieeexplore.ieee.org/abstract/document/9363450)) and dynamic cloning strategy for light-weight estimation performance.
* Multi-sensor simulation toolbox for IMU, camera, LiDAR, GNSS, and wheel enhanced from our prior work ([OpenVINS](https://github.com/rpng/open_vins))
* Evaluation toolbox for consistency, accuracy, and timing analysis.
* Very detailed [options](https://github.com/rpng/mins/tree/master/mins/src/options) for each sensor enabling general multi-sensor application.

## Dependency
MINS is tested on Ubuntu 18 and 20 and only requires corresponding ROS ([Melodic](https://wiki.ros.org/melodic) and [Noetic](https://wiki.ros.org/noetic)).
* Default Eigen version will be 3.3.7 (Noetic) or lower, but if one has a higher version the compilation can be failed due to thirdparty library (libpointmatcher) for LiDAR.

## Build and Source
```
mkdir -p $MINS_WORKSPACE/catkin_ws/src/ && cd $MINS_WORKSPACE/catkin_ws/src/
git clone https://github.com/rpng/MINS
cd .. && catkin build
source devel/setup.bash
```

## Run Examples
### Simulation 
```roslaunch mins simulation.launch cam_enabled:=true lidar_enabled:=true```

![alt text](thirdparty/simulation.png)

### Real-World Dataset
#### Directly reading the ros bag file
```roslaunch mins rosbag.launch config:=kaist/kaist_LC path_gt:=urban30.txt path_bag:=urban30.bag```

![alt text](thirdparty/real_bag.png)

Here are the rosbag files and ground truths we used in the evaluation. To be specific, we used [kaist2bag](https://github.com/tsyxyz/kaist2bag) to convert all sensor readings to rosbag files. All rights reserved to [KAIST urban dataset](https://sites.google.com/view/complex-urban-dataset).
| Rosbag  | GT (txt) | GT (csv) | Rosbag  | GT (txt) | GT (csv) | 
| --- | --- | --- | --- | --- | --- | 
|[urban18.bag](https://drive.google.com/open?id=1HNudS-CVrW8kT1_DB_yyMrnSZVbKGadj&usp=drive_copy)|[urban18.txt](https://drive.google.com/open?id=19mVcgiPKjWLAOqmJBvvVpOT7AEWUBQkv&usp=drive_copy)|[urban18.csv](https://drive.google.com/open?id=1hAq3q20pbAz9xLGD4w2mKWN8drktK-Mg&usp=drive_copy)|[urban19.bag](https://drive.google.com/open?id=19gCsvLBzXHXtPfewUMWJhTtrHhXqG1og&usp=drive_copy)|[urban19.txt](https://drive.google.com/open?id=1haYoCIT442tbpzkQRhQhiN8BlTZTDUEm&usp=drive_copy)|[urban19.csv](https://drive.google.com/open?id=1ASRWW858p-GpFfiZITS1tjXyY7feOm91&usp=drive_copy)|
|[urban20.bag](https://drive.google.com/open?id=1c2ZbE2dCVWjqvnpE2FDyaEg-9eq3aSzR&usp=drive_copy)|[urban20.txt](https://drive.google.com/open?id=1vbX76X8cmlr-bETpC4j35LwFOYrm7K2V&usp=drive_copy)|[urban20.csv](https://drive.google.com/open?id=1zatY4blLVFPWgtjY-M831DaQ95Kq5_ZB&usp=drive_copy)|[urban21.bag](https://drive.google.com/open?id=17pKSBWBnWZCPuqCw1liWcN63yMOsjegy&usp=drive_copy)|[urban21.txt](https://drive.google.com/open?id=1vh8vEhgk54hgBKCdEbabS2S4CsaM_5Gb&usp=drive_copy)|[urban21.csv](https://drive.google.com/open?id=1A1KsegtdkFJ9GPWM5B3_irLTZXUDfH4X&usp=drive_copy)|
|[urban22.bag](https://drive.google.com/open?id=19p_NsZKmLMzhP4ZnYZ8bKeMSsde2kKPl&usp=drive_copy)|[urban22.txt](https://drive.google.com/open?id=1KHKwqv_9WjKWprqVX6GS7IdSz2WjTQvM&usp=drive_copy)|[urban22.csv](https://drive.google.com/open?id=1tCNznrB2WuonmYOcAPJ0hLgX4T2tou-6&usp=drive_copy)|[urban23.bag](https://drive.google.com/open?id=13cnNKxU_-Q95ph9y8l2ZmydWDYdX1M8K&usp=drive_copy)|[urban23.txt](https://drive.google.com/open?id=1Ht-tohq4Bcw83HOuivECa2hSNL8yZsUt&usp=drive_copy)|[urban23.csv](https://drive.google.com/open?id=1u4zknw9v4MGaoREXDlLs0M_q47DNMt3H&usp=drive_copy)|
|[urban24.bag](https://drive.google.com/open?id=17owKYyDWC67RtiPHkMIH4aRUMIX5wwPp&usp=drive_copy)|[urban24.txt](https://drive.google.com/open?id=1murp0mnpGMudcTCg7ANSkC0dedwXNyyc&usp=drive_copy)|[urban24.csv](https://drive.google.com/open?id=1OR6sFViwGKEifNj11apmqYhLogtZEMUd&usp=drive_copy)|[urban25.bag](https://drive.google.com/open?id=1VISLND6208plOh4SbQRyqaEbqB_IdUl8&usp=drive_copy)|[urban25.txt](https://drive.google.com/open?id=1ce4rYqQooEojFWQcEU82y0Xf5JLNZpst&usp=drive_copy)|[urban25.csv](https://drive.google.com/open?id=18zT43ZsK3r6CFUFz-2vN7DazXCNV7eTr&usp=drive_copy)|
|[urban26.bag](https://drive.google.com/open?id=1TXMQc9R5qXc07AAadXN4Vo4ig8fWaoX7&usp=drive_copy)|[urban26.txt](https://drive.google.com/open?id=1WtuneVN-P8Dvcnp3EosothnLw1rn9Xon&usp=drive_copy)|[urban26.csv](https://drive.google.com/open?id=17ueerXJEIO6RxPXRC9WnxsZs3ZSQVhOM&usp=drive_copy)|[urban27.bag](https://drive.google.com/open?id=1bhYZSMxPhlSStJgf8vwN0Wkkv-mFBqmp&usp=drive_copy)|[urban27.txt](https://drive.google.com/open?id=1qpuH3BUp24hlYygJ1ENq9kASRmLwGf2-&usp=drive_copy)|[urban27.csv](https://drive.google.com/open?id=1-oBkkxnE1zeMq5xU9s9AdIJH2SsZOH4z&usp=drive_copy)|
|[urban28.bag](https://drive.google.com/open?id=1jLPQVWvW8IU2VDyAphRPkEntf-7VEuqy&usp=drive_copy)|[urban28.txt](https://drive.google.com/open?id=1GrDSlJekqN7dbVeO3SlZda7iOYspm39v&usp=drive_copy)|[urban28.csv](https://drive.google.com/open?id=1z3LFyRuAIi_J8nwC06-0nxd8X8lMCGZn&usp=drive_copy)|[urban29.bag](https://drive.google.com/open?id=1wXhaMnl7O5YVt2ZAeYt2mvE6EUZbsV9t&usp=drive_copy)|[urban29.txt](https://drive.google.com/open?id=10lLf5JqzK9qDNdQRrt6UL8Nwz3tPc4fb&usp=drive_copy)|[urban29.csv](https://drive.google.com/open?id=1W0Ql2a2jQowuSKRLpj21UlkZ1Mkv4wiV&usp=drive_copy)|
|[urban30.bag](https://drive.google.com/open?id=1RKeFV3NqDunqyra2tvFLwuUncxTeuKtJ&usp=drive_copy)|[urban30.txt](https://drive.google.com/open?id=1e4GRzcHmnhvk6F6m2zSzN6AXIzO5PtOs&usp=drive_copy)|[urban30.csv](https://drive.google.com/open?id=1A-jfRD8ITVJwrBjwY0ZoSB1lykJ0aQHd&usp=drive_copy)|[urban31.bag](https://drive.google.com/open?id=1k_5c_tgMTIoPatSaev68qgi_1ets-9Ne&usp=drive_copy)|[urban31.txt](https://drive.google.com/open?id=1ei1zR9v0DJqWTR9lggA1eUuOScpRsqo-&usp=drive_copy)|[urban31.csv](https://drive.google.com/open?id=1ShJpnJedT2CUUH2WO0YWy0idcvWpzQ-K&usp=drive_copy)|
|[urban32.bag](https://drive.google.com/open?id=1SFTxgVUzK4eph_VeQsx7Tiux-vpmB_7P&usp=drive_copy)|[urban32.txt](https://drive.google.com/open?id=1oRi18bhWwqZWf15vSstjdQxmsboqQ8sy&usp=drive_copy)|[urban32.csv](https://drive.google.com/open?id=1oqut7k-rxQ3GK-4iWAOUeFVAg4o6s4Tm&usp=drive_copy)|[urban33.bag](https://drive.google.com/open?id=1a8FS2eA5rc2gv_Ozh5R4IJXoS4UsJw8Y&usp=drive_copy)|[urban33.txt](https://drive.google.com/open?id=1BEubUr8ZiPv7x9QEFbMxFzLR44hpnF5p&usp=drive_copy)|[urban33.csv](https://drive.google.com/open?id=1MvUTRsbP1MfmufJN7m9G2GnhpcAWChNY&usp=drive_copy)|
|[urban34.bag](https://drive.google.com/open?id=1vYYQ9Kksv8vC07Vhd1vqhjG004NWz22a&usp=drive_copy)|[urban34.txt](https://drive.google.com/open?id=15JZrUYAKu3T8c-IITy-vgOYApWlmnFvm&usp=drive_copy)|[urban34.csv](https://drive.google.com/open?id=1XdODat7Lve9MckvWCuxwTkNDCcg99QWD&usp=drive_copy)|[urban35.bag](https://drive.google.com/open?id=1M7qZjOhp2HYIaNCTNVA4-cmJ7DoBB6H-&usp=drive_copy)|[urban35.txt](https://drive.google.com/open?id=1bNkKppTUIUXcV3JADG6otfC-E03MxEH7&usp=drive_copy)|[urban35.csv](https://drive.google.com/open?id=1ZuupbOuOhR8fQXeqIENRR0CUBGwapdpk&usp=drive_copy)|
|[urban36.bag](https://drive.google.com/open?id=1ksYdYpuFuYLpQt-13YhW3OWEroAsXSgd&usp=drive_copy)|[urban36.txt](https://drive.google.com/open?id=1O0Q4HbNAAkNk2TPi18BQmyHKeIEjaPCA&usp=drive_copy)|[urban36.csv](https://drive.google.com/open?id=1EZDNj4lK5JKBMLtVmwJG533LIVc6SU6Z&usp=drive_copy)|[urban37.bag](https://drive.google.com/open?id=1FjYRs0XDtbY4oVkIYnr8u7P-kE8X8Vs4&usp=drive_copy)|[urban37.txt](https://drive.google.com/open?id=113hoQYBKv_nxgZP8nGKjOVPoqHI21kdB&usp=drive_copy)|[urban37.csv](https://drive.google.com/open?id=1Zw_gsH-JRVPTZp4ynQ-SrvyULssksqxp&usp=drive_copy)|
|[urban38.bag](https://drive.google.com/open?id=1g2QmL1mcGuiJ8M2Dd9d9ha7C3WQGHNmx&usp=drive_copy)|[urban38.txt](https://drive.google.com/open?id=1WfbbjzKMJ4dmG82ZrMm9fE-Ngas3gVmN&usp=drive_copy)|[urban38.csv](https://drive.google.com/open?id=1l6_J-81LJVJRzO7QulRgUwQNd6zifdzZ&usp=drive_copy)|[urban39.bag](https://drive.google.com/open?id=1Dva7dk2zbU4vXH0Nykr8M9VcVpwmLorc&usp=drive_copy)|[urban39.txt](https://drive.google.com/open?id=1HIxZIATwf57e9bWUTu4AnIuOZB4J4YIl&usp=drive_copy)|[urban39.csv](https://drive.google.com/open?id=1AyA2LH6d1vVAs2KP4l1l_2ho7MOhZLEp&usp=drive_copy)|
#### Subscribing to the ros messages
```roslaunch mins subscribe.launch config:=euroc_mav rosbag:=V1_03_difficult.bag bag_start_time:=0```

![alt text](thirdparty/real_sub.png)

### RViz
```rviz -d mins/launch/display.rviz```

## Acknowledgements
This project was built on top of the following libraries which are in the thirdparty folder.
* [OpenVINS](https://github.com/rpng/open_vins): Open-source filter-based visual-inertial estimator.
* [ikd-tree](https://github.com/hku-mars/ikd-Tree): Incremental k-d tree.
* [libpointmatcher](https://github.com/ethz-asl/libpointmatcher): Modular Iterative Closest Point library based on [libnabo](https://github.com/ethz-asl/libnabo)

## Credit / Licensing


This code was written by the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at the
University of Delaware. If you have any issues with the code please open an issue on our GitHub page with relevant
implementation details and references. For researchers that have leveraged or compared to this work, please cite the
following:

The publication reference will be updated soon.

```bibtex
@article{Lee2023arxiv,
    title        = {MINS: Efficient and Robust Multisensor-aided Inertial Navigation System},
    author       = {Woosik Lee and Patrick Geneva and Chuchu Chen and Guoquan Huang},
    year         = 2023,
    journal      = {arXiv preprint arXiv:2309.15390},
    url          = {https://github.com/rpng/MINS},
}
```

The codebase and documentation is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
You must preserve the copyright and license notices in your derivative work and make available the complete source code with modifications under the same license ([see this](https://choosealicense.com/licenses/gpl-3.0/); this is not legal advice).
