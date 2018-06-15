# Attaque:
## file:
attaque.h
## MIN:MAX:
1:1
## Behavior need:
- striker
- search_shoot_area
## Affectation:
- Si le robot est le plus proche de la balle par rapport à ses alliés --> striker.
- Sinon --> search_shoot_area.

# Attaque avec support:
## file:
attaque_with_support.h
## MIN:MAX:
2:2
## Behavior need:
- striker
- support
## Affectation:
- Le plus proche de la balle --> striker.
- L'autre --> support

# Defensive:
## file:
defensive.h
## MIN:MAX:
1:1
## Behavior need:
- Degageur
- Obstructeur
## Affectation:
- Si le robot est le plus proche de la balle par rapport à tous les robots --> Degageur.
- Sinon --> Obstructeur.

# Mur:
## file:
mur.h
## MIN:MAX:
1:2
## Behavior need:
- Degageur
- ConsignFollower
## Affectation:
- Si le robot est le plus proche de la balle et que la balle est à une distance inférieur à un seuil --> Degageur.
- Sinon --> ConsignFollower (Pour garder le ou les robot(s) sur la ligne et entre la balle et les cages).


# FromStrat<goalie>:
## MIN:MAX:
1:1
## Behavior need:
- Goalie
## Affectation:
--> Goalie
