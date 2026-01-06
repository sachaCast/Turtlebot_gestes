# Turtlebot Gesture Control – `ia_turtlebot_vision`

Ce package ajoute un module de **vision** pour contrôler un TurtleBot3 à partir de **gestes** détectés par une caméra, dans un environnement ROS 2. 

---

## Objectifs de cette partie 

- Permettre à un TurtleBot3 de réagir à des **gestes humains** détectés en temps réel via la caméra Raspberry Pi.
- S’intégrer dans une architecture ROS 2 standard (nœuds, topics, packages) compatible avec la simulation virtuelle TurtleBot3 et la démo réelle sur le robot.

---

## Architecture ROS 2

### Structure du workspace

Le package de vision est `ia_turtlebot_vision`, placé dans un workspace ROS 2 (`ia_turtlebot_ws`) :

```
ia_turtlebot_ws/
├── src/
│   └── ia_turtlebot_vision/
│       ├── ia_turtlebot_vision/
│       │   ├── gesture_detector_node.py
│       │   ├── person_detector_node.py
│       │   ├── webcam_publisher_node.py
│       │   └── models/
│       │       ├── gesture_recognizer.task
│       │       ├── MobileNetSSD_deploy.caffemodel
│       │       └── MobileNetSSD_deploy.prototxt.txt
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       └── test/
├── build/        # généré par colcon (ignoré par git)
├── install/      # généré par colcon (ignoré par git)
└── log/          # généré par colcon (ignoré par git)
```