# Projet Turtlebot3 : IA, Vision et Contrôle par Gestes

Ce projet implémente un système de contrôle intelligent pour un **Turtlebot3 Burger** équipé d'une caméra de profondeur (**d435i**) dans l'environnement de simulation Gazebo. Il utilise **MediaPipe** pour la reconnaissance de gestes et un **Superviseur** pour orchestrer les comportements du robot.

## 1. Prérequis Système (Ubuntu 22.04 + ROS 2 Humble)

Avant de lancer le projet, assurez-vous que ROS 2 Humble et les outils de compilation sont installés sur votre machine.

### Installation de ROS 2 et des outils de base

```bash
# Installation de ROS2 Humble Desktop
sudo apt update && sudo apt install ros-humble-desktop -y

# Installation de colcon (compilateur) et des outils de dépendances
sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp -y

# Configuration de l'environnement
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

```

### Configuration des paquets Turtlebot3 et Simulation

Il est crucial d'exporter le bon modèle pour que les fichiers URDF et les transformations (TF) se chargent correctement.

```bash
# Installation des dépendances ROS pour la simulation
sudo apt install ros-humble-ros-gz ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-descriptions ros-humble-turtlebot3-msgs \
                 ros-humble-cv-bridge ros-humble-tf2-ros -y

# Configuration du modèle spécifique avec caméra D435i
echo "export PROJECT_MODEL=turtlebot3_burger_d435i" >> ~/.bashrc
source ~/.bashrc

```

---

## 2. Installation des Dépendances Python (IA)

Le système de vision repose sur MediaPipe et OpenCV pour traiter le flux de la caméra.

```bash
# Installation des bibliothèques de vision
pip install opencv-python mediapipe msgpack

```

---

## 3. Installation du Projet

```bash
# Création de l'espace de travail
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copiez vos fichiers dans ce dossier, puis rendez les scripts exécutables :
chmod +x gesture_detector_node.py person_detector_node.py supervisor.py obstacle_avoider.py changer_images.py

```

---

## 4. Lancement et Utilisation


---

## 5. Guide des Commandes (via changer_images.py)

| Touche | Image Affichée | Action du Robot |
| --- | --- | --- |
| **v** | Victory (✌️) | **Activation** : Passe en mode `LISTENING`. |
| **z** | Thumb Up (👍) | **Avancer** : Le robot avance de 10cm. |
| **s** | Thumb Down (👎) | **Reculer** : Le robot recule de 10cm. |
| **q** | Closed Fist (✊) | **Gauche** : Rotation de 10°. |
| **d** | Pointing Up (☝️) | **Droite** : Rotation de 10°. |
| **e** | Open Palm (✋) | **Stop** : Arrêt et retour au mode `EXPLORING`. |

---
