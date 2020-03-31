# Perception in Robotics course final project (Term3 2020, Skoltech, Moscow-Russia)

## Project team
- Evgeny Tsykunov (evgeny.tsykunov@skolkovotech.ru)
- Valery Ilin (valery.ilin@skoltech.ru)
- Stepan Perminov (stepan.perminov@skoltech.ru)
- Aleksey Fedoseev (aleksey.fedoseev@skoltech.ru)
- Elvira Zainulina (elvira.zainulina@skoltech.ru)


## Requirements
- **python3.7** (cause used methods from python3.7 - \_\_future__)
- ubuntu 18.04
- opencv-python
- pyrealsense2
- matplotlib
- open3d
- pptk
- octomap
- pyglet

*List off all requirements in requirements.txt*

### Python3.7 Ubuntu installation
#### For Linux Users

For installation Python3.7 was used this [link](https://stackoverflow.com/questions/54633657/how-to-install-pip-for-python-3-7-on-ubuntu-18)
```bash

sudo apt install python3-pip
sudo apt install python3.7
python3.7 -m pip install pip

sudo apt install python3.7-venv

python3.7 -m venv venv 
source venv/bin/activate

pip install -r requirements.txt
```
#### For Windows Users

Install python3.7 with next [link](https://www.python.org/downloads/)
```bash
python3.7 -m venv venv 
# then you have to activate venv with your console: https://docs.python.org/3/library/venv.html

pip install -r requirements.txt
```

#### Bags with installing
##### Octomap:
```bash
pip uninstall octomap-python
sudo apt-get install liboctomap*
pip install glooey
pip install imgviz
pip install pyglet
pip install trimesh
pip install octomap-python --no-binary octomap-python
```
##### PPTK launching problem:
```bash
cd venv/lib/python3.7/site-packages/pptk/libs/
mv libz.so.1 libz.so.1.old
sudo ln -s /lib/x86_64-linux-gnu/libz.so.1
```
*If you got any other problems, please write to @valeryilin in Telegram*
## Launching
### Preparing
To launch this project you have to download bag files by [link](https://drive.google.com/open?id=1mgudOdZlnPuyCJGJV6N4l2ZX7HhZN2hI) and paste them to the data folder.

If you want to use real sensor, you have to change Variable is_real in main.py

### Common launching
```bash
python main.py
```

### Visualisation launching
This launching is just for demo because we got a log delay between frames
```bash
python pyglet_demo.py
```

### Notebooks
*All notebooks are in the folder **notebooks***. Use ```jupyter lab``` to launch them.