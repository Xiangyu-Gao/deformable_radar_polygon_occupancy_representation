# Deformable Radar Polygon

A Lightweight and Predictable Occupancy Representation for Short-range Collision Avoidance

<p align="center"> <img src='docs/grap_abs.png' align="center" height="300px"> </p>

> [**Deformable Radar Polygon: A Lightweight and Predictable Occupancy Representation for Short-range Collision Avoidance**](https://arxiv.org/pdf/2203.01442.pdf),            
> *Xiangyu Gao, Sihao Ding, Karl Vanäs, Harshavardhan Reddy Dasari, and Henrik Söderlund*
> *arXiv technical report ([arXiv 2203.01442](https://arxiv.org/abs/2203.01442))*  

    @article{xiangyu2022deformable,
        title={Deformable Radar Polygon: A Lightweight and Predictable Occupancy Representation for Short-range Collision Avoidance},
        author={Xiangyu, Gao and Sihao, Ding and Karl, Vanas and Reddy, Dasari Harshavardhan and Henrik, Soderlund},
        journal={arXiv preprint arXiv:2203.01442},
        year={2022}}

## Update
***(Nov. 27, 2022) Code and sample data release.***

## Contact
Any questions or suggestions are welcome! 

Xiangyu Gao [xygao@uw.edu](mailto:xygao@uw.edu) 

## Abstract
Inferring the drivable area in a scene is a key capability for ensuring the vehicle avoids obstacles and enabling safe autonomous driving. However, a traditional occupancy grid map suffers from high memory consumption when forming a fine-resolution grid for a large map. In this paper, we propose a lightweight, accurate, and predictable occupancy representation for automotive radars working for short-range applications that take interest in instantaneous free space surrounding the sensor. This new occupancy format is a polygon composed of a bunch of vertices selected from noisy radar measurements, which covers free space inside and gives a Doppler moving velocity for each vertex. It not only takes a very small memory and computing resources for storage and updating at every timeslot but also has the predictable shape-change property based on vertex's Doppler velocity. We name this kind of occupancy representation 'deformable radar polygon'. Two formation algorithms for radar polygon are introduced for both single timeslot and continuous inverse sensor model update. To fit this new polygon representation, a matrix-form collision detection method has been modeled as well. The radar polygon algorithms and collision detection model have been validated via extensive experiments with real collected data and simulations, showing that the deformable radar polygon is very competitive in terms of its completeness, smoothness, accuracy, lightweight as well as the shape-predictable property. 

## Use Deformable Radar Polygon

All radar configurations and algorithm configurations are included in [config](config.py).

### Software Requirement and Installation

Python 3.6, and libraries in [requirements.txt](requirements.txt).

### Download Sample Data
1. From below Google Drive link download the sample data 'scene.pickle'. 
    ```
    https://drive.google.com/file/d/12NV46iAPfws4SVUuxyfHbsqE_BibqAyK/view?usp=sharing
    ```
    Note that the sample data is from the scene 143 of [RadarScenes](https://radar-scenes.com/) Dataset. You can try other data by downloading the RadarScenes, and read and save other scenes of data with the [script](read_radarscene_data.py).

2. Relocate the pickle data under the project directory.
    ```
    ./deformable_radar_polygon_occupancy_representation/scene.pickle
    ```

## Run Single-shot Radar Polygon Formation Algorithm
    
    python polygon_radarscene_singleFrame.py
    

## Run ISM-based Radar Polygon Update Algorithm
   
    python polygon_radarscene_inverseSensor.py
    

## Run the Reference Occupancy Grid Algorithm   
    python gridmap_radarScene_ism.py
    

## Evaluation 
    python evaluate_radarScenes.py

The slicing results are the RA slices, RV slices, and VA slices as shown in below figure.
<p align="center"> <img src='docs/slice_viz.png' align="center" height="230px"> </p>

## License

This project is release under MIT license (see [LICENSE](LICENSE)).

## Acknowlegement
This work was partially completed during the internship at Volvo Cars. Thanks so much for all the people who gave guidance or contributed to this project.
