# Le manager a deux plans de jeu :
- le plan défensif
- le plan offensif

On définit une ligne de dangerosité L1.

# Plan défensif :

## Condition d'activation :
Le pan défensif s'active quand la balle est de notre côté de la ligne L1.
## Stratégies assignées :
1 stratégie attaque (avec 1 robot),  2 stratégies defensive (à 1 robot chacun), 1 Mur (à 2 robots), 1 goal (à 1 robot)
## Affectation des rôles au démarrage du plan défensif :
### a) Par défaut:
- Le goal est déjà affecté.
- Les positions des robots des murs et des stratégies défensives sont choisies parmi les robots les plus proches.
- Le dernier robot restant est affecté à l'attaquant.
### b) Si le plan précédent était offensif:
- Le goal est déjà affecté.
- Le robot défensif marquant le joueur de plus faible menace devient stratégie Mur.
- Le robot attaquant devient stratégie defensive (pour remplacer le robot ci-dessus).
- Le deuxième robot defensif ne change pas.
- Le deuxième mur ne change pas.
- Le support devient attaquant.

# Plan offensif :
## Condition d'activation :
Le plan offensif s'active quand la balle est du côté ennemis de la ligne L1.
## Stratégies assignées :
1 stratégie "Attaque avec support" (avec 2 robots), 2 stratégies défensive (à 2 robots), 1 Mur (à 1 robot), 1 goal (à 1 robot)
## Affectation des rôles au demmarage du plan offensif :
### a) Par défaut :
- Le goal est déjà affecté.
- Les positions des robots des stratégie défensives et du mur sont choisies parmi les robots les plus proches.
- Le robot restant le plus proche de la balle devient attaquant.
- Le dernier robot devient support de l'attaquant.
### b) si le plan précédent était défensif :
- Le goal est déjà affecté
- L'ancien attaquant et l'ancien défenseur le plus proche de la balle est affecté à la stratégie "Attaque avec support",
- L'autre robot précédement affecté à la stratégie défenseur reste affecté à la stratégie défenseur
- Le robot du mur le plus proche du defenseur qui a changé de poste est affecté à la stratégie défensive.
- L'autre mur reste affecté au mur
