# ROS-Neuro smr processing package
the package provides the pipeline followed by the IAS-lab, where a laplacian filter, a pwelch algorithm, a ringbuffer and a gaussian classsifier are used as classes. The configuration of the laplacian, ringbuffer and gaussian filter are computed via yaml files as reported in their rosneuro packages, instead for pwelch the parameters are taken as input from command line.

## Requirements
rosneuro_smr_processing has been tested with the following configuration:
- **Ubuntu 20.04.05 LTS Focal Fossa** and **ROS Noetic**

rosneuro_filters depends on:
- [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page)

### Subscribed topic:
- ```/neurodata``` ([rosneuro_msgs/NeuroFrame](https://github.com/rosneuro/rosneuro_msgs))

Input data for the processing.
  
### Published topic:
- ```/smr/neuroprediction``` ([rosneuro_msgs/NeuroOutput](https://github.com/rosneuro/rosneuro_msgs) 

Output data probability after processing and classification have been applied.

### Parameters:
For examples of parameters and yaml files, see the corresponding rosneuro packages.

### Example of launch file
In the current launch file, only acquisition node and smr node are launched.

### Already provided cfg
In the repository *rosneuro_smr_processing/cfg* are stored some useful yaml file for standard cases. Additionally, there are two subdirectories:
- *rosneuro_smr_processing/cfg/16ch* provides some example with 16 channels:
    - *gaussianCfg.yaml*: an examle of a gaussian classifier
    - *laplacian.yaml*: an example of laplacian filter
- *rosneuro_smr_processing/cfg/32ch*
Moreover, it is present an example of a ring buffer in the file *rosneuro_smr_processing/cfg/ringbuffer.yaml*

