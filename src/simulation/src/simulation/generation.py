import matplotlib.pyplot as plt
import numpy as np
import secrets
import json
import math


def rectangle(taille):
    l=np.random.randint(taille*1,taille*2)
    L=np.random.randint(taille*2,taille*3)
    coins.append([0,l])
    coins.append([L,l])
    coins.append([L,0])


def piste_legal(coins):
    if len(coins) < 4:
        return False


    N = len(coins)

    for i in range(N):
        a = coins[i]
        b = coins[(i + 1) % N]

        if a == b:
            return False

        if a[0] != b[0] and a[1] != b[1]:
            return False
        
    for i in range(N):
        avant = coins[i - 1]
        actuel = coins[i]
        apres = coins[(i + 1) % N]

        alignes_verticalement = (
            avant[0] == actuel[0] == apres[0]
        )

        alignes_horizontalement = (
            avant[1] == actuel[1] == apres[1]
        )

        if alignes_verticalement or alignes_horizontalement:
            return False


    rouge = bord_rouge(coins)

    if len(rouge) < 4:
        return False


    for i in range(len(rouge)):
        a = rouge[i]
        b = rouge[(i + 1) % len(rouge)]

        if a == b:
            return False

        if a[0] != b[0] and a[1] != b[1]:
            return False


    for bord in [coins, rouge]:

        N = len(bord)

        for i in range(N):
            a = bord[i]
            b = bord[(i + 1) % N]

            for j in range(i + 1, N):
                c = bord[j]
                d = bord[(j + 1) % N]

                voisins = (
                    j == i + 1
                    or
                    (i == 0 and j == N - 1)
                )

                if a[1] == b[1] and c[1] == d[1]:

                    if a[1] == c[1]:

                        gauche1 = min(a[0], b[0])
                        droite1 = max(a[0], b[0])

                        gauche2 = min(c[0], d[0])
                        droite2 = max(c[0], d[0])
                        if max(gauche1, gauche2) < min(droite1, droite2):
                            return False
                        if (
                            not voisins
                            and
                            max(gauche1, gauche2) == min(droite1, droite2)
                        ):
                            return False
                elif a[0] == b[0] and c[0] == d[0]:

                    if a[0] == c[0]:

                        bas1 = min(a[1], b[1])
                        haut1 = max(a[1], b[1])

                        bas2 = min(c[1], d[1])
                        haut2 = max(c[1], d[1])

                        if max(bas1, bas2) < min(haut1, haut2):
                            return False

                        if (
                            not voisins
                            and
                            max(bas1, bas2) == min(haut1, haut2)
                        ):
                            return False
                else:

                    if a[1] == b[1]:
                        horizontal1 = a
                        horizontal2 = b

                        vertical1 = c
                        vertical2 = d

                    else:
                        horizontal1 = c
                        horizontal2 = d

                        vertical1 = a
                        vertical2 = b


                    croisement = (
                        min(horizontal1[0], horizontal2[0])
                        <= vertical1[0]
                        <= max(horizontal1[0], horizontal2[0])

                        and

                        min(vertical1[1], vertical2[1])
                        <= horizontal1[1]
                        <= max(vertical1[1], vertical2[1])
                    )

                    if croisement and not voisins:
                        return False


    for i in range(len(coins)):

        vert_a = coins[i]
        vert_b = coins[(i + 1) % len(coins)]

        for j in range(len(rouge)):

            rouge_a = rouge[j]
            rouge_b = rouge[(j + 1) % len(rouge)]

            if (
                vert_a[1] == vert_b[1]
                and
                rouge_a[1] == rouge_b[1]
            ):

                if vert_a[1] == rouge_a[1]:

                    gauche_vert = min(vert_a[0], vert_b[0])
                    droite_vert = max(vert_a[0], vert_b[0])

                    gauche_rouge = min(rouge_a[0], rouge_b[0])
                    droite_rouge = max(rouge_a[0], rouge_b[0])

                    if (
                        max(gauche_vert, gauche_rouge)
                        <=
                        min(droite_vert, droite_rouge)
                    ):
                        return False

            elif (
                vert_a[0] == vert_b[0]
                and
                rouge_a[0] == rouge_b[0]
            ):

                if vert_a[0] == rouge_a[0]:

                    bas_vert = min(vert_a[1], vert_b[1])
                    haut_vert = max(vert_a[1], vert_b[1])

                    bas_rouge = min(rouge_a[1], rouge_b[1])
                    haut_rouge = max(rouge_a[1], rouge_b[1])

                    if (
                        max(bas_vert, bas_rouge)
                        <=
                        min(haut_vert, haut_rouge)
                    ):
                        return False

            elif vert_a[1] == vert_b[1]:

                croisement = (
                    min(vert_a[0], vert_b[0])
                    <= rouge_a[0]
                    <= max(vert_a[0], vert_b[0])

                    and

                    min(rouge_a[1], rouge_b[1])
                    <= vert_a[1]
                    <= max(rouge_a[1], rouge_b[1])
                )

                if croisement:
                    return False


            else:

                croisement = (
                    min(rouge_a[0], rouge_b[0])
                    <= vert_a[0]
                    <= max(rouge_a[0], rouge_b[0])

                    and

                    min(vert_a[1], vert_b[1])
                    <= rouge_a[1]
                    <= max(vert_a[1], vert_b[1])
                )

                if croisement:
                    return False

    return True

def nettoyer(coins):
    coins_nettoyes = [coin.copy() for coin in coins]

    changement = True

    while changement:
        changement = False

        for i in range(len(coins_nettoyes)):
            avant = coins_nettoyes[i - 1]
            actuel = coins_nettoyes[i]
            apres = coins_nettoyes[(i + 1) % len(coins_nettoyes)]

            alignes_verticalement = (
                avant[0] == actuel[0] == apres[0]
            )

            alignes_horizontalement = (
                avant[1] == actuel[1] == apres[1]
            )

            if alignes_verticalement or alignes_horizontalement:
                coins_nettoyes.pop(i)
                changement = True
                break

    return coins_nettoyes

def ajouter_couille(taille):
    n=np.random.randint(0,len(coins))
    l=[-coins[n][0]+coins[(n+1)%len(coins)][0],-coins[n][1]+coins[(n+1)%len(coins)][1]]
    # print(n)
    # print(l)
    d=0
    if l[0]==0:
        d=1
    if abs(l[d])<2:
        ajouter_couille(taille)
        return
    a=coins[n].copy()
    b=coins[(n+1)%len(coins)].copy()
    # print(a,b)
    debut = min(a[d], b[d])
    fin = max(a[d], b[d])

    k = np.random.randint(debut, fin)
    j = np.random.randint(k + 1, fin+1)

    if a[d] < b[d]:
        a[d] = k
        b[d] = j
    else:
        a[d] = j
        b[d] = k

    coins.insert(n+1,a.copy())
    coins.insert(n+2,b.copy())

    p=np.random.randint(-taille,taille+1)
    while p==0:
        p=np.random.randint(-taille,taille+1)

    a[(d+1)%2]+=p
    b[(d+1)%2]+=p

    coins.insert(n+2,a)
    coins.insert(n+3,b)

    i=0
    while i<len(coins):
        avant=coins[i-1]
        actuel=coins[i]
        apres=coins[(i+1)%len(coins)]

        if (avant[0]==actuel[0]==apres[0] or
            avant[1]==actuel[1]==apres[1]):
            coins.pop(i)
        else:
            i+=1

def gen_piste(complexité,taille):
    global coins

    coins=[[0,0]]
    rectangle(taille)

    for k in range(complexité):
        mem = [coin.copy() for coin in coins]

        for tentative in range(1000):
            coins = [coin.copy() for coin in mem]

            ajouter_couille(taille)

            if piste_legal(coins):
                break
        else:
            break

    return coins

def direction(coins,i):
    entrée=[-coins[(i-1)%len(coins)][0]+coins[i][0],-coins[(i-1)%len(coins)][1]+coins[i][1]]
    sortie=[coins[(i+1)%len(coins)][0]-coins[i][0],coins[(i+1)%len(coins)][1]-coins[i][1]]
    return(entrée,sortie)

def bord_rouge(coins):
    rouge = []

    for i in range(len(coins)):
        entrée, sortie = direction(coins, i)

        ne = np.linalg.norm(entrée)
        ns = np.linalg.norm(sortie)

        gauche_entrée = [-entrée[1] / ne, entrée[0] / ne ]

        gauche_sortie = [-sortie[1] / ns, sortie[0] / ns]

        x = coins[i][0] + (gauche_entrée[0] + gauche_sortie[0]) * largueur_piste

        y = coins[i][1] + (gauche_entrée[1] + gauche_sortie[1]) * largueur_piste

        rouge.append([x, y])

    return rouge

def placer(coins,couleur):
    blocs=[]
    for k in range(len(coins)):
        entrée,sortie = direction(coins,k)
        d=[-entrée[i]+sortie[i] for i in range(2)]
        #direction 0=droit-haut 1=bas-droit 2=gauche-bas 3=haut-gauche
        direc=0
        if d[0]>0 and d[1]<0:
            direc=1
        if d[0]<0 and d[1]<0:
            direc=2
        if d[0]>0 and d[1]>0:
            direc=0
        if d[0]<0 and d[1]>0:
            direc=3
        # print(direc)
        # print(coins[k])
        bloc={"position":[int(coins[k][0]),int(coins[k][1])], "direction":direc,"couleur":couleur,"type":"virage"}
        blocs.append(bloc)
    for k in range(len(coins)):
        a = coins[k]
        b = coins[(k + 1) % len(coins)]
        ax_, ay_ = int(round(a[0])), int(round(a[1]))
        bx_, by_ = int(round(b[0])), int(round(b[1]))

        if ay_ == by_:                                   # horizontal
            for x in range(min(ax_, bx_) + 1, max(ax_, bx_)):
                blocs.append({"position": [x, ay_], "direction": 0,
                              "couleur": couleur, "type": "mur"})
        else:                                            # vertical
            for y in range(min(ay_, by_) + 1, max(ay_, by_)):
                blocs.append({"position": [ax_, y], "direction": 1,
                              "couleur": couleur, "type": "mur"})
    return blocs





def afficher_blocs(blocs, ax=None):
    import matplotlib.patches as patches

    if ax is None:
        fig, ax = plt.subplots()

    # flèches d'orientation
    dir_mur    = {0: (1, 0),  1: (0, 1)}            # horizontal / vertical
    dir_virage = {0: (0.4, 0.4), 1: (0.4, -0.4),    # droite-haut / bas-droite
                  2: (-0.4, -0.4), 3: (-0.4, 0.4)}  # gauche-bas / haut-gauche

    for bloc in blocs:
        x, y = bloc["position"]
        couleur = bloc["couleur"]

        if bloc["type"] == "mur":
            # case pleine semi-transparente centrée sur la position
            ax.add_patch(patches.Rectangle(
                (x - 0.5, y - 0.5), 1, 1,
                facecolor=couleur, edgecolor="black",
                alpha=0.35, linewidth=0.8))
            dx, dy = dir_mur[bloc["direction"]]
            ax.plot([x - dx * 0.4, x + dx * 0.4],
                    [y - dy * 0.4, y + dy * 0.4],
                    color=couleur, linewidth=2)

        else:  # virage
            # marqueur sur le point du coin + flèche vers l'intérieur
            ax.scatter(x, y, s=120, c=couleur,
                       marker="s", edgecolors="black", zorder=3)
            dx, dy = dir_virage[bloc["direction"]]
            ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                        arrowprops=dict(arrowstyle="->", color=couleur, lw=1.5))

    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    return ax




def afficher_coins(coins):
    for coin in coins:
        x=coin[0]
        y=coin[1]
        plt.scatter(x, y, s=50, c='r')
    plt.show()
    # print(piste_legal(coins))
    
    
def afficher_piste(coins):
    x = [coin[0] for coin in coins]
    y = [coin[1] for coin in coins]
    x.append(x[0])
    y.append(y[0])
    plt.plot(x, y)

    bords_rouges = bord_rouge(coins)
    bords_verts = coins.copy()

    x_rouge = [bord[0] for bord in bords_rouges]
    y_rouge = [bord[1] for bord in bords_rouges]

    x_rouge.append(x_rouge[0])
    y_rouge.append(y_rouge[0])

    plt.plot(x_rouge, y_rouge, color="red")


    x_vert = [bord[0] for bord in bords_verts]
    y_vert = [bord[1] for bord in bords_verts]

    x_vert.append(x_vert[0])
    y_vert.append(y_vert[0])

    plt.plot(x_vert, y_vert, color="green")
    plt.axis('equal')
    plt.show()

def blocs_vers_wbt(blocs):
    centre = {0: (0.5, 0.5), 1: (0.5, -0.5), 2: (-0.5, -0.5), 3: (-0.5, 0.5)}
    rotation = {0: 0.0, 1: -math.pi / 2, 2: math.pi, 3: math.pi / 2}
    couleurs = {
        "r": ("rouge", "0.9 0 0", "1 0 0"),
        "g": ("vert", "0 0.6 0", "0 1 0"),
    }

    noeuds = []
    for i, bloc in enumerate(blocs):
        x, y = bloc["position"]
        nom_couleur, couleur_virage, couleur_mur = couleurs[bloc["couleur"]]

        if bloc["type"] == "virage":
            dx, dy = centre[bloc["direction"]]
            noeuds.append(f"""    Solid {{
      translation {x + dx} {y + dy} 0.185
      rotation 0 0 1 {rotation[bloc["direction"]]}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              ambientIntensity 0.5
              diffuseColor {couleur_virage}
              emissiveColor {couleur_virage}
              specularColor {couleur_virage}
            }}
          }}
          geometry Mesh {{
            url [
              "ImageToStl.com_virage.obj"
            ]
          }}
        }}
      ]
      name "virage_{nom_couleur}_{i}"
      boundingObject Mesh {{
        url [
          "ImageToStl.com_virage.obj"
        ]
      }}
    }}""")
        else:
            angle = 0.0 if bloc["direction"] == 0 else math.pi / 2
            noeuds.append(f"""    Solid {{
      translation {x} {y} 0.1
      rotation 0 0 1 {angle}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {couleur_mur}
            metalness 0
          }}
          geometry Box {{
            size 1 0.1 0.2
          }}
        }}
      ]
      name "droit_1m_{nom_couleur}_{i}"
      boundingObject Box {{
        size 1 0.1 0.2
      }}
    }}""")

    return "\n".join(noeuds)


coins=[[0,0]]
largueur_piste=1.0

def creer_checkpoint(coins):
    rouge = bord_rouge(coins)
    n = len(coins)
    milieu = [
        [(coins[i][0] + rouge[i][0]) / 2, (coins[i][1] + rouge[i][1]) / 2]
        for i in range(n)
    ]

    checkpoints = []
    for i in range(n):
        a = milieu[i]
        b = milieu[(i + 1) % n]
        entree, sortie = direction(milieu, i)
        theta_entree = math.atan2(entree[1], entree[0])
        theta_sortie = math.atan2(sortie[1], sortie[0])
        theta = math.atan2(
            math.sin(theta_entree) + math.sin(theta_sortie),
            math.cos(theta_entree) + math.cos(theta_sortie),
        )
        ux = sortie[0] / math.hypot(*sortie) - entree[0] / math.hypot(*entree)
        uy = sortie[1] / math.hypot(*sortie) - entree[1] / math.hypot(*entree)
        norme = math.hypot(ux, uy)
        decalage = 0.207 if norme > 1e-9 else 0.0
        cx = a[0] + (ux / norme) * decalage if norme > 1e-9 else a[0]
        cy = a[1] + (uy / norme) * decalage if norme > 1e-9 else a[1]
        checkpoints.append([theta, cx, cy])

        longueur = math.hypot(b[0] - a[0], b[1] - a[1])
        nb = math.ceil(longueur - 1e-9)
        theta_ligne = math.atan2(b[1] - a[1], b[0] - a[0])
        for k in range(1, nb):
            t = k / nb
            checkpoints.append(
                [theta_ligne, a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
            )

    return checkpoints


def generer_map(seed=None,complexite=4,taille=2):

    global coins

    if seed is None:
        seed=secrets.randbelow(100)
    print(seed)
    np.random.seed(seed)

    coins=[[0,0]]
    coins=gen_piste(complexite,taille)
    coins=nettoyer(coins)

    bords_rouges = bord_rouge(coins)
    blocs_rouge=placer(bords_rouges,'r')
    blocs_vert=placer(coins,'g')

    return seed, coins, blocs_rouge, blocs_vert


def creer_world_wbt(chemin, seed=None,complexite=4,taille=2):
    seed, coins, blocs_rouge, blocs_vert = generer_map(seed,complexite,taille)
    blocs = blocs_rouge + blocs_vert

    xs = [bloc["position"][0] for bloc in blocs]
    ys = [bloc["position"][1] for bloc in blocs]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    taille_sol = max(10, max(xs) - min(xs) + 4, max(ys) - min(ys) + 4)

    entete = f"""#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
IMPORTABLE EXTERNPROTO "../protos/TT02_2023b.proto"

WorldInfo {{
  basicTimeStep 25.0
}}
Viewpoint {{
  orientation -0.15612198054631546 0.9765966041367921 -0.1479357968132854 1.0925600380817375
  position {cx - 4} {cy} 6.6
  followType "Mounted Shot"
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Group {{
  children [
"""
    pied = f"""
  ]
}}
Solid {{
  translation {cx} {cy} -0.05
  children [
    Shape {{
      appearance PBRAppearance {{
        roughness 1
      }}
      geometry Box {{
        size {taille_sol} {taille_sol} 0.1
      }}
    }}
  ]
  name "sol_piste"
  boundingObject Shape {{
    geometry Box {{
      size {taille_sol} {taille_sol} 0.1
    }}
  }}
}}
DEF WorldInit Robot {{
  supervisor TRUE
  name "WorldInit"
  controller "controller_world_init"
  children [
  ]
}}
"""
    with open(chemin, "w") as f:
        f.write(entete + blocs_vers_wbt(blocs) + pied)

    with open(chemin.replace(".wbt", ".json"), "w") as f:
        json.dump({"seed": seed, "checkpoints": creer_checkpoint(coins)}, f)

    return seed


if __name__ == "__main__":
    seed, coins, blocs_rouge, blocs_vert = generer_map()
    afficher_piste(coins)
    # afficher_blocs(blocs_rouge)
