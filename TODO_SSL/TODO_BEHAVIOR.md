# striker:
## file:
striker.h
## Control:
- charge: 1
- kickPower: 255
- kick: 1
- chipkick: 0
- spin: ?
## Description:
Fonce vers la balle et tire dans le goal adverse.

# goalie:
## file:
goalie.h
## Control:
- charge: 1
- kickPower: 255
- kick: 0
- chipkick: 1
- spin: ?
## Description:
Reste dans la surface de réparation et sur la trajectoire de la balle. Quand la balle est dans la surface de réparation la dégager avec un chipkick.

# support
## file:
support.h
## Control:
- charge: 0
- kickPower: -
- kick: 0
- chipkick: 0
- spin: 0
## Description:
Reste à un vecteur V d'un robot donnée en entrée.

# search_shoot_area:
## file:
search_shoot_area.h
## Control:
- charge: 0
- kickPower: -
- kick: 0
- chipkick: 0
- spin: 0
## Description:
Cherche, dans une zone donnée, une position pour tirer sans avoir de robot sur la trajectoire du tir.

# degageur:
## file:
degageur.h
## Control:
- charge: 1
- kickPower: 255
- kick: 0
- chipkick: 1
- spin: ?
## Description:
Fonce vers la balle et la dégage avec un chipkick (si possible vers un autre robot).

# obstructeur:
## file:
obstructeur.h
## Control:
- charge: 0
- kickPower: -
- kick: 0
- chipkick: 0
- spin: 0
## Description:
Reste sur la droite entre les cages et un robot adverse (à une distance d du robot).
