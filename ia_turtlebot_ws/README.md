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

### Nœuds et topics

| Nœud ROS 2 | Rôle principal | Topics |
|------------|----------------|--------|
| webcam_publisher_node	| Capture la webcam et publie les images | Pub : `/camera/image_raw` (`sensor_msgs/Image`) |
| person_detector_node | Détection de personnes dans l’image via MobileNet SSD | Sub : `/camera/rgb/image_raw` (`sensor_msgs/Image`) / Pub : `/person_detection` (`std_msgs/Float32MultiArray`) |
| gesture_detector_node | Reconnaissance de gestes (MediaPipe) et génération de commandes robot | Sub : `/camera/rgb/image_raw` / Pub : `/gesture_cmd` (`std_msgs/String`) |

Dans ce package, on a donc trois nodes: `webcam_publisher_node`, `person_detector_node` et `gesture_detector_node` (capture, détection, gestes). Néanmoins, seul le node de détecteur de gestes `gesture_detector_node` sera utilisé dans le projet, les deux autres étant été utilisés uniquement pour tester les modèles en local.

---


## Choix techniques

### Modèles de vision

- MobileNet SSD (MobileNetSSD_deploy.caffemodel + .prototxt) pour la détection de personnes.

Modèle léger, adapté au temps réel sur CPU et| largement utilisé dans des exemples pédagogiques.

- MediaPipe Gesture Recognizer (gesture_recognizer.task) pour la reconnaissance de gestes de la main.

Pipeline complète (détection de main, landmarks, classification de gestes) qui simplifie fortement l’implémentation.

Au début du projet, l’objectif était de combiner un modèle de détection de personnes avec le modèle de reconnaissance de gestes, car le système était pensé pour fonctionner avec une caméra 2D.​ Au fil du développement, le choix s’est finalement porté sur une caméra 3D, ce qui a conduit à remplacer cette approche par un modèle de segmentation de nuage de points pour détecter les personnes directement dans les données 3D.

### Mapping gestes → commandes robot

| Geste détecté | Action TurtleBot3 |
|---------------|-------------------|
| Thumb Up | Avancer (tout droit) |
| Thumb Down | Reculer |
| Open Palm | Se stopper |
| Pointing Up | Tourner à droite |
| Closed Fist | Tourner à gauche |

Les gestes choisis sont suffisamment distincts pour limiter les erreurs de classification.

---

## Installation

### Prérequis

- Ubuntu 22.04

- ROS 2 Humble 

- Workspace TurtleBot3 pour la simulation, par exemple :

#### Cloner le dépôt
```
cd ~/Documents/ROS_PROJECT
git clone https://github.com/sachaCast/Turtlebot_gestes.git
cd Turtlebot_gestes/ia_turtlebot_ws
```

#### Build du workspace
```colcon build```

#### Sourcing
```source install/setup.bash```

Les dossiers build/, install/, log/ sont générés automatiquement et ignorés par git grâce au .gitignore. 

### Utilisation
1. Lancer la simulation TurtleBot3

Dans un premier terminal :

```
bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```
et
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Cela démarre Gazebo avec le TurtleBot3 et tous les topics nécessaires.

2. Lancer les nœuds de vision

Dans un deuxième terminal :

```
bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/Documents/ROS_PROJECT/Turtlebot_gestes/ia_turtlebot_ws/install/setup.bash
```

#### Capture webcam (optionnel)
```ros2 run ia_turtlebot_vision webcam_publisher_node```

#### Détection de personnes (optionnel)
```ros2 run ia_turtlebot_vision person_detector_node```

#### Reconnaissance de gestes 
```ros2 run ia_turtlebot_vision gesture_detector_node```
