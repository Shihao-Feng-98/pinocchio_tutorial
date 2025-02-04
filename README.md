# Pinocchio Tutorial
The pinocchio 3.3.1 example.

# Pinocchio Installation
https://github.com/stack-of-tasks/pinocchio

https://stack-of-tasks.github.io/pinocchio/download.html

## Use cpp and python sdk
### Refer `Linux`
The cpp sdk will be installed in the `/opt/openrobots`, and the python sdk will be installed in the ubuntu built-in version.

Ubuntu 18.04 pinocchio 2.6.3
```shell
sudo apt install -qqy robotpkg-py36-pinocchio
```

Ubuntu 20.04 pinocchio 3.3.1
```shell
sudo apt install -qqy robotpkg-py38-pinocchio
```

## Use pincchino in specific python environment 
### Refer `pip`
```python
python -m pip install pin
```

### Refer `conda`
```python
conda install pinocchio -c conda-forge
```