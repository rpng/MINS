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
| Rosbag  | GT (csv) | GT (txt) | Rosbag  | GT (csv) | GT (txt) | 
| --- | --- | --- | --- | --- | --- | 
|[urban18.bag](https://drive.google.com/open?id=1WuFvsgc416E5oi7ROLYy0Y6ekClVARGV&usp=drive_copy)|[urban18.csv](https://drive.google.com/open?id=1rqXJwwex2lfAJrLUZY9fUs98ePzZIKRc&usp=drive_copy)|[urban18.txt](https://drive.google.com/open?id=11rchRZvbDuSywUrDnlOYeBF2woS3GSnC&usp=drive_copy)|[urban19.bag](https://drive.google.com/open?id=1QjGEvMjNsk3fJeQV6GuZ1MiVGvpnMXuL&usp=drive_copy)|[urban19.csv](https://drive.google.com/open?id=1epB0k7YOdMraywCuedowHB7wf6kPYnqd&usp=drive_copy)|[urban19.txt](https://drive.google.com/open?id=1TBsiouJSThTwcMRYyY65gAIY0f3zXhaX&usp=drive_copy)|
|[urban20.bag](https://drive.google.com/open?id=1joISXo2zvBy03gbQzUC_LTSmoIXaZ5u1&usp=drive_copy)|[urban20.csv](https://drive.google.com/open?id=1fpEixMNjoq8gT-qMw0GX0fiULt-lyP3M&usp=drive_copy)|[urban20.txt](https://drive.google.com/open?id=1UObOWam0nI1s4TstRZ_UTaypkaaVV9gA&usp=drive_copy)|[urban21.bag](https://drive.google.com/open?id=17Fq1ixHwPaLwMGZrx2oZp-Aw0piXc7Kv&usp=drive_copy)|[urban21.csv](https://drive.google.com/open?id=1tmXnE6rOnErfuMYhVPZFpNCeJDkNHvVe&usp=drive_copy)|[urban21.txt](https://drive.google.com/open?id=1JZSFhr66V2QEyesrmcyeYDU4O2e7eetV&usp=drive_copy)|
|[urban22.bag](https://drive.google.com/open?id=1PrciTcIyn5URc9iKW5TOQ9V-C6Q_hI4Z&usp=drive_copy)|[urban22.csv](https://drive.google.com/open?id=1wjW8wq-FVno7lo7uljlND4DuA9wNivFV&usp=drive_copy)|[urban22.txt](https://drive.google.com/open?id=1sxOamJJytLVBTzgejGRr3kFqcCpDk7yM&usp=drive_copy)|[urban23.bag](https://drive.google.com/open?id=1ptcc5joAl1LhSoVzf9IgGvDqm93IFoBx&usp=drive_copy)|[urban23.csv](https://drive.google.com/open?id=1HFZpkNBASZ7CEGrjQqTRpnuyedW4mKpL&usp=drive_copy)|[urban23.txt](https://drive.google.com/open?id=1ICzaR6Z5mDkR1KM3vkHejCgdsMJNIjiC&usp=drive_copy)|
|[urban24.bag](https://drive.google.com/open?id=1GqbFoQd6R8kzRX4VqWdtoy5PnEZxTx4O&usp=drive_copy)|[urban24.csv](https://drive.google.com/open?id=16U01BQbWqndbsutXt3iCK2FQ_dZ7kHQ_&usp=drive_copy)|[urban24.txt](https://drive.google.com/open?id=1PTUKyoMkG01MLebI1CTwfp6C6RVlFAkC&usp=drive_copy)|[urban25.bag](https://drive.google.com/open?id=1YWyui_bOQuSnmP_-NPsQ-G39fnElZL_o&usp=drive_copy)|[urban25.csv](https://drive.google.com/open?id=1vZqH2RKrR5OOUVK1fw8RSmsRHa5RnteV&usp=drive_copy)|[urban25.txt](https://drive.google.com/open?id=11mH5fsPBd7kPktLfLaaoKQjYNUbIm1AL&usp=drive_copy)|
|[urban26.bag](https://drive.google.com/open?id=1zyFSLg5nDPGwjOAmqwZIjitJmp7Ate3V&usp=drive_copy)|[urban26.csv](https://drive.google.com/open?id=1sriL5sv-VgjG7T7kFjRFz_MiojmRAGfF&usp=drive_copy)|[urban26.txt](https://drive.google.com/open?id=1ERGCd4RR0vav7Cbb2Agw9UA2AJOE07nQ&usp=drive_copy)|[urban27.bag](https://drive.google.com/open?id=1x5FWI7ds5fFDuz6VoQYOsDoKEzFhQFaA&usp=drive_copy)|[urban27.csv](https://drive.google.com/open?id=1hT2P2AJGggmo6W-oD8Qowqj2lZfuAUzL&usp=drive_copy)|[urban27.txt](https://drive.google.com/open?id=1forf325XmRLRq07rQmOrOnNZEl3HDmt-&usp=drive_copy)|
|[urban28.bag](https://drive.google.com/open?id=14xOmDZ_4ktLZS18ghi7JTPyHXj1awDWr&usp=drive_copy)|[urban28.csv](https://drive.google.com/open?id=1H_xcy5hk9V2i27g3DPxYMqSXIiF6dGBp&usp=drive_copy)|[urban28.txt](https://drive.google.com/open?id=1dFRr7fOySzvmXQcSLamo5Zc_zw_9WHzn&usp=drive_copy)|[urban29.bag](https://drive.google.com/open?id=1sOYQjvCZUDxG4PGuWia7GsrMntdvUqPq&usp=drive_copy)|[urban29.csv](https://drive.google.com/open?id=1hi8E0RptAK4rpbo-eQc4ArFQRy6qO68O&usp=drive_copy)|[urban29.txt](https://drive.google.com/open?id=1mTUr-QwOLXjFx8h5egmhXm4O4n93S9tP&usp=drive_copy)|
|[urban30.bag](https://drive.google.com/open?id=1bw1SJxTQEL74dZTOuD8USnCb5skZ59hY&usp=drive_copy)|[urban30.csv](https://drive.google.com/open?id=1rpLEkMUW1vao854l2mGXKDYRAo48_rFF&usp=drive_copy)|[urban30.txt](https://drive.google.com/open?id=1LCHy_ncIKxgfV6HZy_PUlsEWaczhv3Dk&usp=drive_copy)|[urban31.bag](https://drive.google.com/open?id=1cKRyxgeeWzRGlKJgr2a_GQdOkn7V255H&usp=drive_copy)|[urban31.csv](https://drive.google.com/open?id=1rVigiMX86JT4H9j5y2P9xmDoMkVYx8sj&usp=drive_copy)|[urban31.txt](https://drive.google.com/open?id=14uIkrra4MEez8QV0_GtcUi_8x19vlFb3&usp=drive_copy)|
|[urban32.bag](https://drive.google.com/open?id=1c4fmUX_UBYOW1Y2j0Kr1ViUKBcAb5r59&usp=drive_copy)|[urban32.csv](https://drive.google.com/open?id=16Zk5sZSu7J1LmaKG7Xs6cCUz4x39Ecew&usp=drive_copy)|[urban32.txt](https://drive.google.com/open?id=1K_FwA9_RQxlz3W9NXycrceaZDIoQuUlC&usp=drive_copy)|[urban33.bag](https://drive.google.com/open?id=1tdaIB2xz7o2bfDOLySJijn9Bdk7v_v0V&usp=drive_copy)|[urban33.csv](https://drive.google.com/open?id=1ERMSVuqFBUTaHqOhQi9gKpCEH9092b7l&usp=drive_copy)|[urban33.txt](https://drive.google.com/open?id=1Gzy-hq6UkA-G0S7mEd1--5QQqmev_fAC&usp=drive_copy)|
|[urban34.bag](https://drive.google.com/open?id=1_K-Q_KfLvj8-xrJy2RL4XMOh3nXQmPAe&usp=drive_copy)|[urban34.csv](https://drive.google.com/open?id=10VveQ2LJ0hNEn3WZ76GdKAU_VdHV24cC&usp=drive_copy)|[urban34.txt](https://drive.google.com/open?id=1ie7KFQC3q9LBsViHkYnU6n1eQZv18ayw&usp=drive_copy)|[urban35.bag](https://drive.google.com/open?id=10oUZ1u7kgxHYjCQGNRApvt7nMl15Ll4V&usp=drive_copy)|[urban35.csv](https://drive.google.com/open?id=1L046y2g6iBCvTovA94Fh9UAussXx6lZ2&usp=drive_copy)|[urban35.txt](https://drive.google.com/open?id=1X_TuHUt8sLRN9DPjAEEqydyxtGVnvXzb&usp=drive_copy)|
|[urban36.bag](https://drive.google.com/open?id=1DWEnJ3pluJmNQUtGABsELftqyO7GzxL4&usp=drive_copy)|[urban36.csv](https://drive.google.com/open?id=1A14eAcsBgXaKpwwPRoyArxPRSwAcU3TW&usp=drive_copy)|[urban36.txt](https://drive.google.com/open?id=1b1bTU780RcHq-wiFvwIx8bqPypFDt252&usp=drive_copy)|[urban37.bag](https://drive.google.com/open?id=1v27W0Baw2iLdzu7Rwh0g_KMhf4FfUGP9&usp=drive_copy)|[urban37.csv](https://drive.google.com/open?id=1-USD8fNa-Dif0riGh21CymQh0_l9mMCb&usp=drive_copy)|[urban37.txt](https://drive.google.com/open?id=1KOakD6gNawaW0iD8VLg58GeujzHqMpcf&usp=drive_copy)|
|[urban38.bag](https://drive.google.com/open?id=1LJEi4whPwONl61NAp54gaJjDBZkCwk_Q&usp=drive_copy)|[urban38.csv](https://drive.google.com/open?id=1-EVRyxGsSrQyrxZvP5faD9oYbScbkGe9&usp=drive_copy)|[urban38.txt](https://drive.google.com/open?id=1mcD4AfFUSQHK5KAPmu2-nSvvGL1NufI6&usp=drive_copy)|[urban39.bag](https://drive.google.com/open?id=1JmdHv_GWeHEFGJj37UXV9KF3iQKTHLm-&usp=drive_copy)|[urban39.csv](https://drive.google.com/open?id=1CGRIAg8i_xyJdaWDn93n_IWfjfPiZPjU&usp=drive_copy)|[urban39.txt](https://drive.google.com/open?id=1TYjr1dK6a2wgSIWpKimvwPIAR_sr4-fF&usp=drive_copy)|
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
