This is a project for parsing opendrive .xodr file and visualization using matplotlib.

# Introductions

- /opendriveparser: Copied from [https://github.com/fiefdx/pyopendriveparser](https://github.com/fiefdx/pyopendriveparser) for parsing opendrive .xodr files. Note that some bugs are fixed in this work.
- /data: Consisting input demo data of .xodr file.
- parse_and_visualize.py: Parsing and visualizing the .xodr file.

# Installation

Please create virtual environment and install required packages.

```
conda create -n opendrive python=3.8
conda activate opendrive
pip install lxml>=5.1.0
pip install matplotlib
pip install https://github.com/stefan-urban/pyeulerspiral/archive/master.zip
```

# Parameters and Run

The parameters are given at the start of “parse_and_visualize.py” including:

- XODR_FILE: input file path.
- XXXX_COLOR: colors of different lane types.
- STEP: Sample steps while ploting.

Directly run the main file:

```
python parse_and_visualize.py
```

The results of visualization of the road network will be saved in “/data/” directory.

If you find this demo useful, please consider star our work!
