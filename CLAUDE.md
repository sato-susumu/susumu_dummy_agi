# susumu_dummy_agi

ROS2�ñ��(n���ա��

## ���Ȃ�

Sn�ñ��oROS2��ïn���_��ЛY����AGI�ñ��gY

## �����L��

### ���
```bash
# ROS2�������n���ǣ���g�L
cd /home/taro/ros2_ws
colcon build
```

### �L
```bash
# ������
source install/setup.bash

# �����ɒ�L
ros2 run susumu_dummy_agi relay_node
```

## �ñ���

- `susumu_dummy_agi/relay_node.py`: ��n�����ɟ�
- `test/`: ƹ�ա��
- `package.xml`: ROS2�ñ����
- `setup.py`: Python ��Ȣ��ա��

## �z��

- colcon buildo�Z `/home/taro/ros2_ws` g�LY�Sh
- ��ɢ�ƣա��build/, install/, log/	ogitignorek+~�fD~Y