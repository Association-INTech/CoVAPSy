# Lidar

<img src="../img/UST-10lx.jpg" alt="an image showing a UST-10lx with an orange top. We can read Hokuyo Smart Urg underneath" title="A UST-10lx Lidar" width="200" align="right"/>


Les Lidar utiliser par INTech sont des [UST-10LX](https://www.hokuyo-aut.jp/search/single.php?serial=167) concu et vendu par Hokuyo.<!-- Ce sont des lidar haut performance. --> Une documentation non officel (mais plutot complete [existe ici])(https://sourceforge.net/p/urgnetwork/wiki/Home/). Le manuel
du l'UST-10lx est [télécharchable ici](./Instruction_manual_UST-10LX_MRS0020D_en_1513910662-1.pdf)

## UST-10lx connectivity


Il communique par Ethernet en utilisant le protocole [Secure Communications Interoperability Protocol (SCIP)](https://en.wikipedia.org/wiki/Secure_Communications_Interoperability_Protocol). Ce protocol peut faire peur a premiere vue mais nous n'utilison que les commande decrit [ici](https://sourceforge.net/p/urgnetwork/wiki/scip_en/). Pour communiquer par ethernet, les lidar possede une adresse IP: 

```
192.168.0.10:10940/24
```

Il faut donc choisir une adresse ip dans le bon sous-resaux. Nous avons arbitrairement choisi:

```
192.168.0.11
```

## UST-10lx specs

<img src="../img/UST-10LX_Scan_pattern.png" alt="an image showing a UST-10lx with an orange top. We can read Hokuyo Smart Urg underneath" title="A UST-10lx Lidar" width="200" align="right"/>


| Spec                | Value                | Unit |
|---------------------|----------------------|------|
| Min distance        | 20                   | mm   |
| Max distance        | 30000                | mm   |
| Accuracy            | ±40                  | mm   |
| Scan angle          | 270                  | °    |
| Angular resolution  | 0.25                 | pt/° |
| Angular resolution  | 1440                 | pt/tr|
| Scan rate           | 40                   | Hz   |
| Poids               | 130                  | g    |
| Voltage             | 10-30                | VDC  |
| Current @24VDC      | 150 (450 at startup) | mA   |


## Using HokuyoReader class

This class is from [cassc/tcp_hokuyo.py](https://gist.github.com/cassc/26ac479624cb028b2567491a68c34fb8)
This class is simpler than [hokuyolx](#using-hokuyolx-class)

Create the class instance with
``` python
sensor = HokuyoReader(IP, PORT) 
```

use `HokuyoReader.stop()` to get rid of any leftover data or problems from an improper shutdown and start mesuring with `HokuyoReader.startContinuous(0, 1080)` with 0 and 1080 the steps that are mesured

```pyhton
sensor.stop()
sensor.startContinuous(0, 1080)
```

Distances can be retrived as a numpy array with the `HokuyoReader.rDistance()` (r standing for radial)

```python
distance_array=sensor.rDistance
```

Use `HokuyoReader.stop()` to gracefully shutdown the lidar

```pyhton
sensor.stop()

```

## Using Hokuyolx class 

This class comes from [SkoltechRobotics/hokuyolx](https://github.com/SkoltechRobotics/hokuyolx). 
This class has considerably more options than [HokuyoReader](#using-hokuyoreader-class) but is more complicated to understand. This class is documented at [http://hokuyolx.rtfd.org/](http://hokuyolx.rtfd.org/). As of 12/12/24 it doesn't work out off the box and we use [HokuyoReader](#using-hokuyoreader-class)