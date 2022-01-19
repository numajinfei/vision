<!--
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-12-20 13:23:58
 * @LastEditors: hw
 * @LastEditTime: 2022-01-12 14:35:32
-->
# 1. production category
featureA:binocular
- camera basler
- motor encoder
  
# 2. vision
vision probe project，including code testing, build, CI/CD and release

# 3. docker run command
## 1. docker container start:
- ros2_test:
  ```shell
  # contain basler:
  docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ~/vision/log:/ws/ros2_log/ --restart unless-stopped --name test jadehu/ros2_feature_a_test
  ```

- ros2_deploy:
  ```shell
  # contain basler:
  docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ~/vision/log:/ws/ros2_log --restart unless-stopped --name deploy jadehu/ros2_feature_a_deploy
  ```
## 2. docker enter the running container
```shell
docker exec -it <container_name/alias_name> /bin/bash
```
