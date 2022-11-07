# ROS Blog Post

Example repository accompanying my blog post [here](https://ben.bolte.cc/ros).

## Getting Started

1. Clone this repository somewhere:

```bash
git clone git@github.com:codekansas/ros-blog-post.git
```

2. Create and activate a new Conda environment:

```bash
CONDA_ENV_NAME=ros-blog-post
conda create --name $CONDA_ENV_NAME python=3.9
conda activate $CONDA_ENV_NAME
```

3. See the Makefile for further instructions:

```bash
$ make
                    ROS Blog Post
                    -------------

# Installing

1. Install ROS on the robot using `make install-ros`
2. Build the package using `make build-ros`
3. Run Turtlesim using `make run-turtlesim`
4. Run the custom Turtlesim node in another terminal using `make run-custom-turtlesim-node`

# Running Static Analysis

1. Install formatting packages using `make install-format`
2. Run autoformatters using `make format`
3. Run static analysis checks using `make static-checks`
```

