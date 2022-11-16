# Vision based End-to-end autonomous driving using Cycle-consistent VAE
The vision based end-to-end autonomous driving framework for CARLA 0.8 benchmarks.

This repository contains the following modules
 1. Disentanglement_VAE: To disentangling domain-specific feature and domain-general feature from pair images using Cycle-consistent VAE.
 2. starlab_2022: To predict the action values to drive an ego-vehicle to the destination based on the Resnet backbone, 
                                  domain-general feature, and conditional imitation learning.


## Getting Started

### Dependencies
* Major dependencies
  1. Python 3.7
  2. Pytorch 1.6
  3. cuda 10.2
 
### Installing
* Importing an uploaded Anaconda environment (torch.yaml) is recommended

### Database Acquisition
* Method for acquisition of driving data on CARLA simulator is described in this [repository](https://github.com/carla-simulator/data-collector).

### CARLA Simulator and Benchmarks
* You can download from this [document](https://carla.org/2018/04/23/release-0.8.2/).

### Executing program
* First Stage: Training cycle-consistent VAE 
 1. Collecting pair images using CARLA Simulator
 2. Move to Disentanglement_VAE
 3. Modify the database path variables (train_pair, eval_pair) in train_CycleVAE_lusr_v2.py
 4. Run below command
```
python train_CycleVAE_lusr_v2.py --id="ID for this training"
```
 5. The trained weights are saved at save_models/id/id.pth

* Second Stage: 


