# SimulationMoteurCC

## Questions

1) Dans le controle de la position pourquoi est ce que j'ai des oscillations alors même que j'utilise un proportionnel donc normalement un controlleur d'ordre 1 ? 

2) le controle des turtlebot je ne comprends pas trop. Je comprends que l'objectif est de controler le bot en controlant l'orientation des roues avec j'imagine le controleur réalisé un peu avant. En revanche qu'estce qu'il entend par vitesse imposées ? qui plus est pourquoi la vitesse est imposée avant même qu'on nous demande d'ajouter les moteurs ?

## Réponses

1) Quand je controlle en vitesse j'utilise effectivement un ordre 1 si je n'active que KP mais en position j'ajoute déjà un intégrateur pour passer de la vitesse à la position. C'est donc normal d'avoir des oscillations.

2) 


IL m'a dit que d'abord l'utilisateur devait donner les vitesses des roues genre en entrée puis que ensuite avec les moterus il doit donner les coord operationnel genre la position final ou on veut aller et notre controleur doit gerer les vitesses rot/trans pour y aller 

