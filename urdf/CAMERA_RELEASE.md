# 相机 URDF 更新

本次更新包含 15 款相机的 URDF 和所引用的 STL，其中 12 款附 CameraInfo YAML。15 份均通过 ROS 2 Jazzy 的 `check_urdf`，并检查了网格引用。URDF 不包含 USD 的 ROS 2 发布图。

| 范围 | 光学帧与标定状态 |
| --- | --- |
| SHF3G、SHF3L、SG2 AR0233 H100F1A/H60FA、SG2 OX03CC H60FA、SG3S H190XA、SG5 H110SA、SG8S H120YA/H30YA/H60SA | 保留与源验收 USD 一致的机身到光学帧关节，并附内参 YAML。未做实物外参测量。 |
| SYF2G_1120、SYW2G | 附内参 YAML；实际镜头位姿待确认，不导出临时光学帧。 |
| S36A、S56D、S56Y | 仅交付 URDF 和网格；缺内外参，不导出光学帧或占位标定。 |

后五款沿用源 USD 的尺寸，机械尺寸尚未验收。原始标定文件和 CAD 中间导出未纳入本次更新。M3A、S56C 不在本次更新范围内。