# navigation

## Urgent for miniwalkie demo

run this on host pc

run slam
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

run nav2
```
ros2 launch robot_navigation localization_Nav2.launch.py nav2_config:=./src/navigation/nav2/config/nav2/nav2_RPP_niggy_real_params.yaml use_sim_time:=False

```

ssh and check log of microRos and etc.
```
ssh niggy@niggy-desktop.lan
journalctl -u robot_bringup.service -f
```


ssh and restart Bringup.
```
ssh niggy@niggy-desktop.lan
sudo systemctl restart robot_bringup.service
```
