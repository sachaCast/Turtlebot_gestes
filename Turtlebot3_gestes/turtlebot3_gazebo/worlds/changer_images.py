import os
import time

IMAGE_PATH = os.path.dirname(os.path.abspath(__file__))
WORLD_NAME = "default"
MODEL_NAME = "mon_panneau_image"
MODEL_NAME_HUMAIN = "panneau_humain"
CMD_PREFIX = "ign"

# Poses pour chaque panneau
POSE_STRING = "7 -7 0.85 0 0 3.14159"
POSE_STRING_HUMAIN = "7.2 -7 0.9 0 0 3.14"

sdf_template = """
<sdf version='1.6'>
    <model name='{name}'>
        <static>true</static>
        <pose>{pose}</pose>
        <link name='link'>
          <visual name='visual'>
            <geometry>
              <box>
                <size>{size}</size>
              </box>
            </geometry>
            <material>
              <ambient>1 1 1 1</ambient>
              <diffuse>1 1 1 1</diffuse>
              <specular>0 0 0 1</specular>
              <pbr>
                <metal>
                  <albedo_map>{texture_path}</albedo_map>
                  <roughness>0.6</roughness>
                  <metalness>0.0</metalness>
                </metal>
              </pbr>
            </material>
          </visual>
          <collision name='collision'>
            <geometry>
              <box>
                <size>{size}</size>
              </box>
            </geometry>
          </collision>
        </link>
    </model>
</sdf>
"""

def update_panel(image_filename, model_name=MODEL_NAME, pose=POSE_STRING, size="0.01 0.4 0.4"):
    """Met à jour un panneau avec une nouvelle image"""
    if image_filename:  # Si une image est fournie
        full_path = os.path.join(IMAGE_PATH, image_filename)

        if not os.path.exists(full_path):
            print(f"ERREUR : Image introuvable : {image_filename}")
            return

        texture_path = full_path
        print(f"Update {model_name} vers : {image_filename}")
    else:  # Si pas d'image (retirer l'image)
        texture_path = ""
        print(f"Retrait de l'image sur {model_name}")

    # 1. Suppression de l'ancien panneau
    delete_cmd = (
        f"{CMD_PREFIX} service -s /world/{WORLD_NAME}/remove "
        f"--reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean "
        f"--req 'name: \"{model_name}\", type: MODEL' --timeout 2000 > /dev/null 2>&1"
    )
    os.system(delete_cmd)

    time.sleep(0.1)

    # 2. Création du nouveau panneau (avec ou sans texture)
    new_sdf = sdf_template.format(
        name=model_name,
        pose=pose,
        texture_path=texture_path,
        size=size
    )
    new_sdf_flat = new_sdf.replace('\n', ' ')

    spawn_cmd = (
        f'{CMD_PREFIX} service -s /world/{WORLD_NAME}/create '
        f'--reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean '
        f'--req "sdf: \\"{new_sdf_flat}\\"" --timeout 2000 > /dev/null'
    )

    os.system(spawn_cmd)

    if image_filename:
        print(f"Image changée sur {model_name} !")
    else:
        print(f"Panneau {model_name} sans image (blanc) !")

def main():
    print("=== Contrôle Panneaux ===")
    print("\n--- Panneau 1 (gestes) ---")
    print("z: forward | s: backward | e: stop | q: left | d: right | v: victory")
    print("x: RETIRER l'image du panneau 1 (panneau blanc)")
    print("\n--- Panneau 2 (humain) ---")
    print("h: AFFICHER test_humain.png")
    print("j: RETIRER l'image du panneau 2 (panneau blanc)")
    print("\nr: quitter\n")

    while True:
        choice = input("Commande > ").strip().lower()

        # Panneau 1 - Gestes
        if choice == 'z':
            update_panel("thumb_up.png")
        elif choice == 's':
            update_panel("thumb_down.png")
        elif choice == 'e':
            update_panel("open_palm.png")
        elif choice == 'q':
            update_panel("closed_fist.png")
        elif choice == 'd':
            update_panel("pointing_up.png")
        elif choice == 'v':
            update_panel("victory.png")
        elif choice == 'x':
            update_panel(None, MODEL_NAME, POSE_STRING, "0.01 0.4 0.4")

        # Panneau 2 - Humain
        elif choice == 'h':
            update_panel("test_humain.png", MODEL_NAME_HUMAIN, POSE_STRING_HUMAIN, "0.02 0.6 1.8")
        elif choice == 'j':
            update_panel(None, MODEL_NAME_HUMAIN, POSE_STRING_HUMAIN, "0.02 0.6 1.8")

        # Quitter
        elif choice == 'r':
            break
        else:
            print("Touche inconnue")

if __name__ == "__main__":
    main()