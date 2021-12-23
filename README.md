<!--
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-12-20 13:23:58
 * @LastEditors: hw
 * @LastEditTime: 2021-12-23 11:14:34
-->
# 1. vision
vision probe project，including code testing, build, CI/CD and release

# 2. docker run command:
## 1. docker container start:
- ros2_test:
    ```shell
    # contain basler:
    docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ./vison/log:/ros2_log/ --restart unless-stopped --name test docker_ros2_test
    ```

- ros2_deploy:
    ```shell
    # contain basler:
    docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ./vison/log:/ros2_log/ --restart unless-stopped --name deploy docker_ros2_deploy
    ```
## 2. docker enter the running container
```shell
docker exec -it <container_name> /bin/bash
```
 