# Makefile

define HELP_MESSAGE
                    Home Robot
                    ----------

# Installing

1. Install ROS on the robot using `make install-ros`
2. Launch ROS using `make launch-ros`

# Running Static Analysis

1. Install formatting packages using `make install-format`
2. Run autoformatters using `make format`
3. Run static analysis checks using `make static-checks`

endef
export HELP_MESSAGE

all:
	@echo "$$HELP_MESSAGE"
.PHONY: all

# ------------------------ #
#        Initialize        #
# ------------------------ #

# Ensures that the user is in an Anaconda environment,
# and that Mamba is installed.
initialize:
ifeq (, $(shell which mamba))
ifeq (, $(shell which conda))
	$(error Conda is not installed)
else
ifeq (base, $(CONDA_DEFAULT_ENV))
	$(error Don't install this package into the base environment. Run 'conda create --name home-robot python=3.8' then 'conda activate home-robot`)
else ifeq (, $(CONDA_DEFAULT_ENV))
	$(error Conda is installed, but not initialized. Run 'conda init'`)
else
	conda install -c conda-forge mamba
endif
endif
endif
.PHONY: initialize

# ------------------------ #
#           ROS            #
# ------------------------ #

install-ros: initialize
	@ mamba install \
		-c robostack-humble \
		-c conda-forge \
		spdlog=1.9.2 \
		foonathan-memory=0.7.2 \
		ros-humble-desktop
.PHONY: install-ros

build-ros: initialize
	colcon build
.PHONY: build-ros

run-ros: initialize
	. install/setup.sh && ros2 run custom_turtlesim controller
.PHONY: run-ros

# ------------------------ #
#         Linting          #
# ------------------------ #

install-format: initialize
	@ mamba install \
		cmake-format \
		clang-format
	@ pip install \
		black \
		darglint \
		flake8 \
		isort \
		mypy_extensions \
		mypy \
		pylint \
		types-setuptools
.PHONY: install-linting

py-files := $$(git ls-files '*.py')
cpp-files := $$(git ls-files '*.c' '*.cpp' '*.h' '*.hpp' '*.cu' '*.cuh')
cmake-files := $$(git ls-files '*/CMakeLists.txt')

format: initialize
	cmake-format -i $(cmake-files)
	clang-format -i $(cpp-files)
	black $(py-files)
	isort $(py-files)
.PHONY: format

static-checks: initialize
	black --diff --check $(py-files)
	isort --check-only $(py-files)
	mypy $(py-files)
	flake8 --count --show-source --statistics $(py-files)
	pylint $(py-files)
.PHONY: static-checks
