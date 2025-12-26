# SimulationMoteurCC

## Questions

1) Dans le controle de la position pourquoi est ce que j'ai des oscillations alors même que j'utilise un proportionnel donc normalement un controlleur d'ordre 1 ? 

2) le controle des turtlebot je ne comprends pas trop. Je comprends que l'objectif est de controler le bot en controlant l'orientation des roues avec j'imagine le controleur réalisé un peu avant. En revanche qu'estce qu'il entend par vitesse imposées ? qui plus est pourquoi la vitesse est imposée avant même qu'on nous demande d'ajouter les moteurs ?

## Réponses

1) Quand je controlle en vitesse j'utilise effectivement un ordre 1 si je n'active que KP mais en position j'ajoute déjà un intégrateur pour passer de la vitesse à la position. C'est donc normal d'avoir des oscillations.
