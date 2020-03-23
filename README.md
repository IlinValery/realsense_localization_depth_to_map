# Perception in Robotics course final project (Term3 2020, Skoltech, Moscow-Russia)

## Project team
- Evgeny Tsykunov (evgeny.tsykunov@skolkovotech.ru)
- Valery Ilin (valery.ilin@skoltech.ru)
- Stepan Perminov (stepan.perminov@skoltech.ru)
- Aleksey Fedoseev (aleksey.fedoseev@skoltech.ru)
- Elvira Zainulina (elvira.zainulina@skoltech.ru)


## Requirements
- **python3.7** (cause used methods from python3.7 - \_\_future__)
- opencv-python
- pyrealsense2
- matplotlib


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