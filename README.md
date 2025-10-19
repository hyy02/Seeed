# Seeed Contest

### 1.Update
- 9.16 : 添加orin端torch, torchvision, realsense手动编译脚本 (Yiyang)
- 9.14 : 添加 VR teleop code (Yiyang)
### 2.Task List

[任务列表](https://docs.qq.com/doc/DTFN6WUtBZ3NGaG1Z?isNewEmptyDoc=1&electronTabTitle=%E7%A9%BA%E7%99%BD%E6%96%87%E6%A1%A3&no_promotion=1&nlc=1)

### 3.Remote source

根据[教程](https://blog.csdn.net/sxf1061700625/article/details/135761567)，加入组网`45b6e887e2f4f9a2`，加入后请联系一洋授权，可以把jetson设备和服务器都连在一个组网里互相访问

- AGX Orin (32G)： IP : `10.147.17.121`, User: `cics` Passwd: `cas-123`
- 由于jetson本身只有64G, 已加装2T固态硬盘: 路径`/media/cics/EFI1/data`

```
python /home/cics/Desktop/codes/projects/lerobot/src/lerobot/scripts/lerobot_record.py \
  --robot.type=xlerobot \
  --robot.cameras="{ head: {type: intelrealsense, serial_number_or_name: 935422072196, width: 640, height: 480, fps: 30, use_depth: True}, right: {type: opencv, index_or_path: '/dev/video6', width: 640, height: 480, fps: 20}, left: {type: opencv, index_or_path: '/dev/video8', width: 640, height: 480, fps: 20} }" \
  --dataset.repo_id=YiyangHuang/eval_table \
  --dataset.single_task="Clear the table." \
  --dataset.root=/media/cics/T7/datasets/eval_table_act \
  --display_data=true \
  --policy.path=/media/cics/T7/checkpoints/pi05_table_pretrained/

```
