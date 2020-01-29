# Package for processing UrbanLoco Dataset 

## Abstract
Mapping and localization is a critical module of autonomous driving, and significant achievements have been reached in this field. Beyond Global Navigation Satellite System (GNSS), research in point cloud registration, visual feature matching, and inertia navigation has greatly enhanced the accuracy and robustness of mapping and localization in different scenarios. However, highly urbanized scenes are still challenging: LIDAR- and camera-based methods perform poorly with numerous dynamic objects; the GNSS-based solutions experience signal loss and multipath problems; the inertia measurement units (IMU) suffer from drifting. Unfortunately, current public datasets either do not adequately address this urban challenge or do not provide enough sensor information related to mapping and localization. Here we present UrbanLoco: a mapping/localization dataset collected in highly-urbanized environments with a full sensor-suite. The dataset includes 13 trajectories collected in San Francisco and Hong Kong, covering a total length of over 40 kilometers. Our dataset includes a wide variety of urban terrains: urban canyons, bridges, tunnels, sharp turns, etc. More importantly, our dataset includes information from LIDAR, cameras, IMU, and GNSS receivers.   

Keywords: Mpapping, Localization, Urban Areas 

<p align="center">
  <img width="pix" src="img/ppp1.png">
</p>


## Usage
- install some additional library
  ```
  $ sudo pip install pykml
  ```
- spancpt2kml.py
  - run and save the trajectory of span-cpt to .kml file
    ```
    python spancpt2kml.py
    ```
- ublox2kml.py
  - run and save the trajectory of u-blox to .kml file
    ```
    python ublox2kml.py
    ```

## Build 
```bash
cd ~/catkin_ws/src
git clone https://github.com/weisongwen/UrbanLoco
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
