# Seeed Contest

### 1.Update
- 9.16 : 添加orin端torch, torchvision, realsense手动编译脚本 (Yiyang)
- 9.14 : 添加 VR teleop code (Yiyang)
### 2.Task List

[任务列表](https://docs.qq.com/doc/DTFN6WUtBZ3NGaG1Z?isNewEmptyDoc=1&electronTabTitle=%E7%A9%BA%E7%99%BD%E6%96%87%E6%A1%A3&no_promotion=1&nlc=1)

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
