# Hyper-ViNL
Hyperparameter study for quadruped robots locomotion with Reinforcement Learning.

This repository contains the work done as part of my Robotics Master's Thesis. The project builds upon the ViNL implementation by Kareer et al. [[1]](#1), which adapts the Legged Gym framework by Rudin et al. [[2]](#2) for obstacle avoidance in indoor environments using the Aliengo Unitree robot.

We have replicated the first two training stages:
1) A **general purpose locomotion** policy which is trained in a rough terrain identical to the one used in [[2]](#2).
2) An **obstacles avoidance locomotion** policy refined from the previous in a flat but cluttered environment.

Then we have conducted an hyperparameter study. The baseline

![Fitting_example](./figures/hyper_param_study_.png)




## Content
- [Hyper-ViNL](#hyper-vinl)
  - [Content](#content)
  - [Source code](#source-code)
    - [1. Auto-Launch](#1-auto-launch)
    - [2. Plotting](#2-plotting)
    - [3. Fitting](#3-fitting)
  - [Acknowledgments](#acknowledgments)
  - [References](#references)

## Source code
### 1. Auto-Launch


### 2. Plotting



### 3. Fitting



## Acknowledgments


## References

<a id="1">[1]</a> 
**Kareer, S., Yokoyama, N., Batra, D., Ha, S., & Truong, J. (2023).**  *ViNL: Visual navigation and locomotion over obstacles.*  arXiv preprint arXiv:2210.14791.  Available at [https://arxiv.org/abs/2210.14791](https://arxiv.org/abs/2210.14791)

<a id="2">[2]</a> 
**Rudin, N., Hoeller, D., Reist, P., & Hutter, M. (2022).**  *Learning to walk in minutes using massively parallel deep reinforcement learning.*  arXiv preprint arXiv:2109.11978. 
Available at [https://arxiv.org/abs/2109.11978](https://arxiv.org/abs/2109.11978)
