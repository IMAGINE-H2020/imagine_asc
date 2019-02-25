# Introduction to ASC
This repo includes the nodes of ASC module. ASC module is responsible for detecting the affordances and providing the action parametes for execution. The message and service files used in this module are provided in imagine_common repo (See: [imagine_common](https://github.com/IMAGINE-H2020/imagine_common.git) ). 

# Requirements
* Common repo that includes messages and services should be included in the workspace [imagine_common](https://github.com/IMAGINE-H2020/imagine_common)

* Perception module (imagine_state_estimation) should be running. (For installation and details please see: [imagine_state_estimation](https://github.com/IMAGINE-H2020/imagine_state_estimation))

* Some of the scripts require weights learned after training. The weigths should be put inside the imagine_asc folder. These weigts are available [here](https://drive.google.com/drive/folders/1vaPicIv9qmm_K3UOmeH0jeA8NSt1y6Zm?usp=sharing)

* 

# Installation
No installation is needed to run this module.

# Running
The service is run with the following line of code.
```
rosrun imagine_asc asc.py
```
# Test
The following line that prints either an error or service result can be run to test the package.
```
    rosrun imagine_asc aff_tester.py
```    
