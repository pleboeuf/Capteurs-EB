# Capteurs-EB

Code pour tous les Capteurs _EB_ de l'Érablière Brunelle

Il suffit de choisir la configuration correspondante à la fonctionnalité disponible et voulu
et modifier la valeur __#define DEVICE_CONF X__ (ligne 45) du fichier Capteurs-EB.ino
et sélectionner un "device" à programmer.

Chaque configuration correspond à une combinaison des senseurs et/ou fonctions suivantes:

| Senseurs/fonctions | Valeur                |
| ------------------ |:---------------------:|
| DISTANCESENSOR:    | US100 / MB7389 / NONE |
| PUMPMOTORDETECT    | true / false          |
| HASDS18B20SENSOR   | true / false          |
| HASHEATING         | true / false          |
| HASVACUUMSENSOR    | true / false          |
| HASVALVES          | true / false          |
| HASRELAYOUTPUT     | true / false          |
| HASUS100THERMISTOR | true / false          |
| baseSampling       | 1 ou 2                |

Les _configurations_ suivantes sont utilisé actuellement.

|Device|DEVICE_CONF|
|------|:------:|
|P1, P2, P3 | 0 |
|V1, V2, V3 | 1 |
|PT1, PT2 | 2 |
|RS1, RS2, RS3, RS4, RF2 | 3 |
|RF1, RC1, RC2 | 4 |
|RS5, RS6 | 5 |
|RHC | 6 |
|VEcTk | 7 |
