# NR-SLAM: Non-Rigid Monocular SLAM

### V0.1, June 28th, 2023
**Authors:** [Juan J. Gómez Rodríguez](https://jj-gomez.github.io/), [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardós](http://webdiis.unizar.es/~jdtardos/).

NR-SLAM is a novel monocular deformable SLAM system founded on the combination of a **Dynamic Deformation Graph** with a **Visco-Elastic deformation model**.
It is able to reconstruct medical imagery with surfaces with different types of topologies and deformations and can use **pinhole** and **fisheye** cameras.

We provide an example workflow to run NR-SLAM with the Andromeda rosbag dataset. Videos of some example executions can be found [here](https://drive.google.com/file/d/12KNHVLE05uoO4x9eZ-qHlGtQ-JPZaAnD).

> For an up-to-date, code-first operational summary (including Andromeda workflow and theory-vs-code drift notes), see [AGENT_CONTEXT.md](AGENT_CONTEXT.md).

<a href="https://youtu.be/N-N0ugRjR2s" target="_blank"><img src="https://youtu.be/N-N0ugRjR2s/0.jpg"
alt="NR-SLAM" width="240" height="180" border="10" /></a>



### Related Publications:
[NR-SLAM] Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **NR-SLAM: Non-Rigid Monocular SLAM**, *ArXiV xxx.yyy*. **[PDF](https://arxiv.org/abs/2308.04036)**.

[Deformable tracking] Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **Tracking monocular camera pose and deformation for SLAM inside the human body**, *IEEE/RSJ International Conference on Intelligent Robots and Systems 2022*. **[PDF](https://arxiv.org/abs/2204.08309)**.

# 1. License

NR-SLAM is released under [AGPL license](https://github.com/endomapper/NR-SLAM/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/endomapper/NR-SLAM/Dependencies.md).

For a closed-source version of NR-SLAM for commercial purposes, please contact the authors: jjgomez (at) unizar (dot) es, josemari (at) unizar (dot) es, tardos (at) unizar (dot) es.

If you use NR-SLAM in an academic work, please cite:

    @article{NR-SLAM,
      title={{NR-SLAM}: Non-Rigid Monocular {SLAM}},
      author={G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={ArXiV xxx.yyy},
      year={2023}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 20.04.4 LTS** but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure good performance and provide more stable and accurate results.

## C++17
We use several functionalities of C++17.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

## Eigen3
Required by g2o. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## Boost
We use [Boost](https://www.boost.org/) for directory operations.

## MLPACK
We use [MLPACK](https://www.mlpack.org/) for clustering operations.

# 3. Building NR-SLAM library and examples

Clone the repository:
```
git clone https://github.com/endomapper/NR-SLAM NR_SLAM
```

We provide a script `build.sh` to build the *third_party* libraries and *NR-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd NR-SLAM
chmod +x build.sh
./build.sh
```

This will create **libNR-SLAM_d**  at *build/lib* folder and the executables in *build/bin* folder.

# 4. Andromeda Example

Andromeda execution is driven by compressed ROS image messages from a rosbag and a secondary state topic.

1. Build the Andromeda target:
```
./build.sh --target andromeda
```

2. Execute the following command:
```
./build/bin/andromeda \
        --dataset_path /home/galactus/Documents/robot-bags/rosbag2_13-02-2026_08-57-17 \
        --settings_path ./data/andromeda/settings.yaml \
        --starting_frame 1771001959417433296 \
        --end_frame 1771002836540509598 \
        --range_mode timestamp_ns \
        --max_images 200 \
        --log_file slam_run.log
```

3. Relevant runtime topics in the rosbag:
         - Main image topic: `/image_raw/compressed`
         - Secondary state topic: `/robot/state`
