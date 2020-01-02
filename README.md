# lepi-ros-server
lepi-ros-server

## 启动docker镜像
`docker run -idt --net host -v ~:/pi  --privileged --name lepi_server wupanhao/lepi_server:melodic`

## 启动服务
`docker restart lepi_server && docker exec -t lepi_server bash -c "source env.sh && roslaunch pi_cam camera_lib_node.launch"  > /tmp/duckie.log &`