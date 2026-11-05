#import matplotlib.pyplot as plt
import numpy as np
import secrets
import json
import math


def rectangle(size):
    l=np.random.randint(size*1,size*2)
    L=np.random.randint(size*2,size*3)
    corners.append([0,l])
    corners.append([L,l])
    corners.append([L,0])


def track_legal(corners):
    if len(corners) < 4:
        return False


    N = len(corners)

    for i in range(N):
        a = corners[i]
        b = corners[(i + 1) % N]

        if a == b:
            return False

        if a[0] != b[0] and a[1] != b[1]:
            return False
        
    for i in range(N):
        before = corners[i - 1]
        current = corners[i]
        after = corners[(i + 1) % N]

        vertically_aligned = (
            before[0] == current[0] == after[0]
        )

        horizontally_aligned = (
            before[1] == current[1] == after[1]
        )

        if vertically_aligned or horizontally_aligned:
            return False


    red = red_corner(corners)

    if len(red) < 4:
        return False


    for i in range(len(red)):
        a = red[i]
        b = red[(i + 1) % len(red)]

        if a == b:
            return False

        if a[0] != b[0] and a[1] != b[1]:
            return False


    for corner in [corners, red]:

        N = len(corner)

        for i in range(N):
            a = corner[i]
            b = corner[(i + 1) % N]

            for j in range(i + 1, N):
                c = corner[j]
                d = corner[(j + 1) % N]

                neighbor = (
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
                            not neighbor
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
                            not neighbor
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


                    crossing = (
                        min(horizontal1[0], horizontal2[0])
                        <= vertical1[0]
                        <= max(horizontal1[0], horizontal2[0])

                        and

                        min(vertical1[1], vertical2[1])
                        <= horizontal1[1]
                        <= max(vertical1[1], vertical2[1])
                    )

                    if crossing and not neighbor:
                        return False


    for i in range(len(corners)):

        green_a = corners[i]
        green_b = corners[(i + 1) % len(corners)]

        for j in range(len(red)):

            red_a = red[j]
            red_b = red[(j + 1) % len(red)]

            if (
                green_a[1] == green_b[1]
                and
                red_a[1] == red_b[1]
            ):

                if green_a[1] == red_a[1]:

                    gauche_vert = min(green_a[0], green_b[0])
                    droite_vert = max(green_a[0], green_b[0])

                    gauche_rouge = min(red_a[0], red_b[0])
                    droite_rouge = max(red_a[0], red_b[0])

                    if (
                        max(gauche_vert, gauche_rouge)
                        <=
                        min(droite_vert, droite_rouge)
                    ):
                        return False

            elif (
                green_a[0] == green_b[0]
                and
                red_a[0] == red_b[0]
            ):

                if green_a[0] == red_a[0]:

                    bas_vert = min(green_a[1], green_b[1])
                    haut_vert = max(green_a[1], green_b[1])

                    bas_rouge = min(red_a[1], red_b[1])
                    haut_rouge = max(red_a[1], red_b[1])

                    if (
                        max(bas_vert, bas_rouge)
                        <=
                        min(haut_vert, haut_rouge)
                    ):
                        return False

            elif green_a[1] == green_b[1]:

                crossing = (
                    min(green_a[0], green_b[0])
                    <= red_a[0]
                    <= max(green_a[0], green_b[0])

                    and

                    min(red_a[1], red_b[1])
                    <= green_a[1]
                    <= max(red_a[1], red_b[1])
                )

                if crossing:
                    return False


            else:

                crossing = (
                    min(red_a[0], red_b[0])
                    <= green_a[0]
                    <= max(red_a[0], red_b[0])

                    and

                    min(green_a[1], green_b[1])
                    <= red_a[1]
                    <= max(green_a[1], green_b[1])
                )

                if crossing:
                    return False

    return True

def clean(corners):
    cleaned_corners = [corner.copy() for corner in corners]

    changed = True

    while changed:
        changed = False

        for i in range(len(cleaned_corners)):
            before = cleaned_corners[i - 1]
            current = cleaned_corners[i]
            after = cleaned_corners[(i + 1) % len(cleaned_corners)]

            vertically_aligned = (
                before[0] == current[0] == after[0]
            )

            horizontally_aligned = (
                before[1] == current[1] == after[1]
            )

            if vertically_aligned or horizontally_aligned:
                cleaned_corners.pop(i)
                changed = True
                break

    return cleaned_corners

def add_turns(size):
    n=np.random.randint(0,len(corners))
    l=[-corners[n][0]+corners[(n+1)%len(corners)][0],-corners[n][1]+corners[(n+1)%len(corners)][1]]
    # print(n)
    # print(l)
    d=0
    if l[0]==0:
        d=1
    if abs(l[d])<2:
        add_turns(size)
        return
    a=corners[n].copy()
    b=corners[(n+1)%len(corners)].copy()
    # print(a,b)
    start = min(a[d], b[d])
    end = max(a[d], b[d])

    k = np.random.randint(start, end)
    j = np.random.randint(k + 1, end+1)

    if a[d] < b[d]:
        a[d] = k
        b[d] = j
    else:
        a[d] = j
        b[d] = k

    corners.insert(n+1,a.copy())
    corners.insert(n+2,b.copy())

    p=np.random.randint(-size,size+1)
    while p==0:
        p=np.random.randint(-size,size+1)

    a[(d+1)%2]+=p
    b[(d+1)%2]+=p

    corners.insert(n+2,a)
    corners.insert(n+3,b)

    i=0
    while i<len(corners):
        before=corners[i-1]
        current=corners[i]
        after=corners[(i+1)%len(corners)]

        if (before[0]==current[0]==after[0] or
            before[1]==current[1]==after[1]):
            corners.pop(i)
        else:
            i+=1

def gen_piste(complexity,size):
    global corners

    corners=[[0,0]]
    rectangle(size)

    for k in range(complexity):
        mem = [coin.copy() for coin in corners]

        for tentative in range(1000):
            corners = [coin.copy() for coin in mem]

            add_turns(size)

            if track_legal(corners):
                break
        else:
            break

    return corners

def direction(coins,i):
    entry=[-coins[(i-1)%len(coins)][0]+coins[i][0],-coins[(i-1)%len(coins)][1]+coins[i][1]]
    exit=[coins[(i+1)%len(coins)][0]-coins[i][0],coins[(i+1)%len(coins)][1]-coins[i][1]]
    return(entry,exit)

def red_corner(coins):
    rouge = []

    for i in range(len(coins)):
        entry, exit = direction(coins, i)

        ne = np.linalg.norm(entry)
        ns = np.linalg.norm(exit)

        gauche_entrée = [-entry[1] / ne, entry[0] / ne ]

        gauche_sortie = [-exit[1] / ns, exit[0] / ns]

        x = coins[i][0] + (gauche_entrée[0] + gauche_sortie[0]) * track_width

        y = coins[i][1] + (gauche_entrée[1] + gauche_sortie[1]) * track_width

        rouge.append([x, y])

    return rouge

def place(corners,color):
    blocs=[]
    for k in range(len(corners)):
        entry,exit = direction(corners,k)
        d=[-entry[i]+exit[i] for i in range(2)]
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
        bloc={"position":[int(corners[k][0]),int(corners[k][1])], "direction":direc,"couleur":color,"type":"virage"}
        blocs.append(bloc)
    for k in range(len(corners)):
        a = corners[k]
        b = corners[(k + 1) % len(corners)]
        ax_, ay_ = int(round(a[0])), int(round(a[1]))
        bx_, by_ = int(round(b[0])), int(round(b[1]))

        if ay_ == by_: # horizontal
            for x in range(min(ax_, bx_) + 1, max(ax_, bx_)):
                blocs.append({"position": [x, ay_], "direction": 0,
                              "couleur": color, "type": "mur"})
        else: # vertical
            for y in range(min(ay_, by_) + 1, max(ay_, by_)):
                blocs.append({"position": [ax_, y], "direction": 1,
                              "couleur": color, "type": "mur"})
    return blocs




"""
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

    bords_rouges = red_corner(coins)
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
"""
def blocks_to_wbt(blocs):
    center = {0: (0.5, 0.5), 1: (0.5, -0.5), 2: (-0.5, -0.5), 3: (-0.5, 0.5)}
    rotation = {0: 0.0, 1: -math.pi / 2, 2: math.pi, 3: math.pi / 2}
    colors = {
        "r": ("rouge", "0.9 0 0", "1 0 0"),
        "g": ("vert", "0 0.6 0", "0 1 0"),
    }

    nodes = []
    for i, bloc in enumerate(blocs):
        x, y = bloc["position"]
        color_name, turn_color, wall_color = colors[bloc["couleur"]]

        if bloc["type"] == "virage":
            dx, dy = center[bloc["direction"]]
            nodes.append(f"""    Solid {{
      translation {x + dx} {y + dy} 0.185
      rotation 0 0 1 {rotation[bloc["direction"]]}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              ambientIntensity 0.5
              diffuseColor {turn_color}
              emissiveColor {turn_color}
              specularColor {turn_color}
            }}
          }}
          geometry Mesh {{
            url [
              "ImageToStl.com_virage.obj"
            ]
          }}
        }}
      ]
      name "virage_{color_name}_{i}"
      boundingObject Mesh {{
        url [
          "ImageToStl.com_virage.obj"
        ]
      }}
    }}""")
        else:
            angle = 0.0 if bloc["direction"] == 0 else math.pi / 2
            nodes.append(f"""    Solid {{
      translation {x} {y} 0.1
      rotation 0 0 1 {angle}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {wall_color}
            metalness 0
          }}
          geometry Box {{
            size 1 0.1 0.2
          }}
        }}
      ]
      name "droit_1m_{color_name}_{i}"
      boundingObject Box {{
        size 1 0.1 0.2
      }}
    }}""")

    return "\n".join(nodes)


corners=[[0,0]]
track_width=1.0

def create_checkpoints(corners):
    red = red_corner(corners)
    n = len(corners)
    middle = [
        [(corners[i][0] + red[i][0]) / 2, (corners[i][1] + red[i][1]) / 2]
        for i in range(n)
    ]

    checkpoints = []
    for i in range(n):
        a = middle[i]
        b = middle[(i + 1) % n]
        entree, sortie = direction(middle, i)
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


def generate_map(seed=None,complexity=4,size=2):

    global corners

    if seed is None:
        seed=secrets.randbelow(100)
    print(seed)
    np.random.seed(seed)

    corners=[[0,0]]
    corners=gen_piste(complexity,size)
    corners=clean(corners)

    red_corners = red_corner(corners)
    red_blocks=place(red_corners,'r')
    green_blocks=place(corners,'g')

    return seed, corners, red_blocks, green_blocks


def create_world_wbt(path, seed=None,complexity=4,size=2):
    seed, coins, red_blocks, green_blocks = generate_map(seed,complexity,size)
    blocs = red_blocks + green_blocks

    xs = [bloc["position"][0] for bloc in blocs]
    ys = [bloc["position"][1] for bloc in blocs]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    ground_size = max(10, max(xs) - min(xs) + 4, max(ys) - min(ys) + 4)

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
        size {ground_size} {ground_size} 0.1
      }}
    }}
  ]
  name "sol_piste"
  boundingObject Shape {{
    geometry Box {{
      size {ground_size} {ground_size} 0.1
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
    with open(path, "w") as f:
        f.write(entete + blocks_to_wbt(blocs) + pied)

    with open(path.replace(".wbt", ".json"), "w") as f:
        json.dump({"seed": seed, "checkpoints": create_checkpoints(coins)}, f)

    return seed


if __name__ == "__main__":
    seed, corners, red_blocks, green_blocks = generate_map()
    #afficher_piste(corners)
    # afficher_blocs(red_blocks)
