# SimulationMoteurCC

## Questions

1) Dans le controle de la position pourquoi est ce que j'ai des oscillations alors même que j'utilise un proportionnel donc normalement un controlleur d'ordre 1 ? 

## Réponses

1) Quand je controlle en vitesse j'utilise effectivement un ordre 1 si je n'active que KP mais en position j'ajoute déjà un intégrateur pour passer de la vitesse à la position. C'est donc normal d'avoir des oscillations.
