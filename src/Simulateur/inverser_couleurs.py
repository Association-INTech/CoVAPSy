import os
import re

# Fichiers à modifier
FILES_TO_MODIFY = [
    "src/Simulateur/worlds/piste2.wbt",
    "src/Simulateur/worlds/piste3.wbt"
]


def swap_colors_precise(file_path):
    if not os.path.exists(file_path):
        print(f"Erreur : Fichier manquant -> {file_path}")
        return

    print(f"Traitement de : {file_path}...")

    with open(file_path, 'r') as f:
        content = f.read()

    # Liste des champs de couleur à modifier
    # (baseColor pour PBR, diffuse/emissive/specular pour Material)
    fields = r"(baseColor|diffuseColor|emissiveColor|specularColor|recognitionColors)"

    # --- DÉFINITION DES VALEURS EXACTES À CIBLER ---

    # GROUPE 1 : MATÉRIAUX (Virages)
    # Rouge Matériau (0.9 0 0)
    regex_mat_red = rf"({fields}\s+(?:\[\s+)?)0\.9\s+0(\.0+)?\s+0(\.0+)?(?!\.)"
    # Vert Matériau (0 0.6 0)
    regex_mat_green = rf"({fields}\s+(?:\[\s+)?)0(\.0+)?\s+0?\.6[0-9]*\s+0(\.0+)?(?!\.)"

    # GROUPE 2 : PBR (Lignes droites)
    # Rouge PBR (1 0 0)
    regex_pbr_red = rf"({fields}\s+(?:\[\s+)?)1(\.0+)?\s+0(\.0+)?\s+0(\.0+)?(?!\.)"
    # Vert PBR (0 1 0) - Attention à ne pas confondre avec 0 0.6 0
    regex_pbr_green = rf"({fields}\s+(?:\[\s+)?)0(\.0+)?\s+1(\.0+)?\s+0(\.0+)?(?!\.)"

    # --- ÉTAPE 1 : ROUGE -> PLACEHOLDER ---
    # On remplace les Rouges par des valeurs temporaires impossibles (99x)

    # 0.9 0 0 -> 991 991 991
    content = re.sub(regex_mat_red, r"\1 991 991 991", content)
    # 1 0 0   -> 992 992 992
    content = re.sub(regex_pbr_red, r"\1 992 992 992", content)

    # --- ÉTAPE 2 : VERT -> ROUGE ---
    # On remplace les Verts par les valeurs Rouges correspondantes

    # 0 0.6 0 -> 0.9 0 0
    content = re.sub(regex_mat_green, r"\1 0.9 0 0", content)
    # 0 1 0   -> 1 0 0
    content = re.sub(regex_pbr_green, r"\1 1 0 0", content)

    # --- ÉTAPE 3 : PLACEHOLDER -> VERT ---
    # On remplace les placeholders (anciens rouges) par du Vert

    # Ancien Rouge Matériau (991) -> Vert Matériau (0 0.6 0)
    content = content.replace("991 991 991", "0 0.6 0")
    # Ancien Rouge PBR (992) -> Vert PBR (0 1 0)
    content = content.replace("992 992 992", "0 1 0")

    with open(file_path, 'w') as f:
        f.write(content)

    print(f"  -> Succès ! Couleurs inversées (Matériaux 0.9/0.6 et PBR 1/1 gérés).")


if __name__ == "__main__":
    print("--- DÉBUT DE L'INVERSION PRÉCISE ---")
    for file_wbt in FILES_TO_MODIFY:
        swap_colors_precise(file_wbt)
    print("--- FIN ---")