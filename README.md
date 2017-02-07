# Capteurs-EB

Code pour tous les Capteurs _EB_ de l'Érablière Brunelle

Il suffit de choisir la configuration correspondante à la fonctionnalité disponible et voulu
et modifier la valeur __#define DEVICE_CONF X__ (ligne 45) du fichier Capteurs-EB.ino
et sélectionner un "device" à programmer.

Chaque configuration correspond à une combinaison des senseurs et/ou fonctions suivantes:

| Senseurs/fonctions | Valeur          |
| ------------------ |:---------------:|
| DISTANCESENSOR:    | US100 ou MB7389 |
| PUMPMOTORDETECT    | true / false    |
| HASDS18B20SENSOR   | true / false    |
| HASHEATING         | true / false    |
| HASVACUUMSENSOR    | true / false    |
| HASVALVES          | true / false    |
| HASRELAYOUTPUT     | true / false    |


Les _configurations_ suivantes sont utilisé actuellement.

P1, P2, P3 -> DEVICE_CONF == 0
V1, V2, V3 -> DEVICE_CONF == 1
PT1, PT2 -> DEVICE_CONF == 2
RS1, RS2, RS3, RS4, RF2 -> DEVICE_CONF == 3
RF1, RC1, RC2 -> DEVICE_CONF == 4
RS5, RS6 -> DEVICE_CONF == 5
RHC -> DEVICE_CONF == 6
