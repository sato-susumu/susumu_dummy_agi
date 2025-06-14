# susumu_dummy_agi

ROS2ÑÃ±ü¸(náâêÕ¡¤ë

## ×í¸§¯È‚

SnÑÃ±ü¸oROS2ÈÔÃ¯nêìü_ı’Ğ›Y‹ÀßüAGIÑÃ±ü¸gY

## ÓëÉûŸL¹Õ

### ÓëÉ
```bash
# ROS2ïü¯¹Úü¹nëüÈÇ£ì¯ÈêgŸL
cd /home/taro/ros2_ws
colcon build
```

### ŸL
```bash
# °ƒ’½ü¹
source install/setup.bash

# êìüÎüÉ’ŸL
ros2 run susumu_dummy_agi relay_node
```

## ÑÃ±ü¸Ë

- `susumu_dummy_agi/relay_node.py`: á¤ónêìüÎüÉŸÅ
- `test/`: Æ¹ÈÕ¡¤ë¤
- `package.xml`: ROS2ÑÃ±ü¸š©
- `setup.py`: Python »ÃÈ¢Ã×Õ¡¤ë

## ‹záâ

- colcon buildoÅZ `/home/taro/ros2_ws` gŸLY‹Sh
- ÓëÉ¢üÆ£Õ¡¯Èbuild/, install/, log/	ogitignorek+~ŒfD~Y