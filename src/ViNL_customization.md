# Installation Instructions

## ViNL GitHub Repo
Git clone the [ViNL GitHub repository](https://github.com/SimarKareer/ViNL):
```
git clone --recurse-submodules git@github.com:SimarKareer/ViNL.git
```
Alternatively, tou can fork their repo in your Github profile and git clone the forked repo so all changes will be saved.

## Miniconda
Install `miniconda` from this link: https://docs.anaconda.com/miniconda/

## Setup.sh
Run the `setup.sh` executable. If you find any problem you always can run line by line the content of the file.

Personally, I recommend to create an alias for the virtual environment, something like the following (modify it as convenient).
```
alias vinl="conda activate vinl && cd ~/Documents/TFM/ViNL && export LD_LIBRARY_PATH=~/miniconda3/envs/vinl/lib/"
```

## Running simulations

The full training `locomotion policy` can be done easily with the help of `my_train.sh` file. Before running it remember to make it executable:
```
chmod +x my_train.sh
```
Now will be as easy as typing:
```
sh my_train.sh
```


## Troubleshooting

1) To avoid any problems regardin the *argument parser* you will need to manually edit the `helpers.py` file. It is located in the `legged_gym/utils` folder. In the line `213`, within the `--checkpoint` argument, you need to change the `type` from `int` to `str`.
2) The following command has been included in the top of `my_train.sh`: ``` export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json ```
3) At the time to record frames when evaluating a trained model, it will be necessary to change lines from `167` to `172` in the `legged_gym/scripts/play.py` file. Basically you need to include the correct path depending on your ViNL folder location. Also change to `True` the `RECORD_FRAMES` global variable (at the end of the same file). Finally, for a proper recording of the robot's movements uncomment the code line `46` that says: `env.follow_cam` and comment the one behind (`env.floating_cam,`).
4) Also install `FFmpeg` within your virtual environment with
    ```
    conda install -c conda-forge ffmpeg
    ```
    because if not installed you will have a problem when recording videos regarding the `Unknown encoder 'libx264'`.
5) Please do not forget to edit all the `aliengo_#_config.py` files to "clean" the `runner`. All of them can look like this:
    ```
    class runner(LeggedRobotCfgPPO.runner):
        alg = "ppo"
        run_name = "rough"
        experiment_name = "rough_aliengo"

        max_iterations = 3500
    ```
    for the others change the names y parameters properly {rough, obstacles, lbc}.
6) It is also necessary to create the next structure: `exported/frames` in your `run_name` folder within the `logs` folder. And the same should be done for the rest of training stages. Here an example for two stages {rough and obstacles}:
    ```
    ViNL
    |- logs
       |- rough_aliengo
       |   |- exported
       |      |- frames
       |- obs_aliengo
          |- exported
             |- frames
    ```
    but for convenience we have included this in `my_train.sh` file, with the following lines:
    ```
    if [ ! -d "$TARGET_DIR" ]; then
        mkdir -p "$DIR"
    fi
    ```
    It is highly recommended to uncomment the line `174`: `print("saving at fp: ", filename)` from the `play.py` file.

<!-- 
7) Could be convenient to include a new parser option in the `legged_gym/utils/helpers.py` file (in line `217`):
    ```
    {
        "name": "--resume_path",
        "type": str,
        "help": "Path from which to load the weights for the training, when resume=True.",
    },
    ```
    and will be also necessary to include in the `helpers.py` file the next (in line `156`):
    ```
    if args.resume_path is not None:
        cfg_train.runner.resume_path = args.resume_path
    ``` 
-->

7) In the `legged_gym/utils/logger.py` file there is a function called `plot()` which could be really helpful but... utilices a lot of memory and makes the training execution do not work properly. **We recomend not using it when launching `my_train.sh`**. But could be helpful when launching indiviual training stages.

8) If you found a message like this (that is originated due to the ffmpeg installation):
   ```
   [libprotobuf FATAL google/protobuf/stubs/common.cc:83] This program was compiled against version 3.6.1 of the Protocol Buffer runtime library, which is not compatible with the installed version (3.20.1). Contact the program author for an update. If you compiled the program yourself, make sure that your headers are from the same version of Protocol Buffers as your link-time library. (Version verification failed in "bazel-out/k8-opt/genfiles/tensorflow/core/framework/tensor_shape.pb.cc".) terminate called after throwing an instance of 'google::protobuf::FatalException' what(): This program was compiled against version 3.6.1 of the Protocol Buffer runtime library, which is not compatible with the installed version (3.20.1). Contact the program author for an update. If you compiled the program yourself, make sure that your headers are from the same version of Protocol Buffers as your link-time library. (Version verification failed in "bazel-out/k8-opt/genfiles/tensorflow/core/framework/tensor_shape.pb.cc".) Aborted (core dumped)
   ``` 
   the following pip commands have finally solved the problem for me:
    ```
    pip install tensorflow==1.15.0
    pip install tensorboard==1.15.0

    pip install protobuf==3.8.0

    pip install torch
    pip install webdataset msgpack objectio
    pip install moviepy decorator proglog
    pip install tensorflow-estimator mock
    pip install keras-applications h5py
    ```