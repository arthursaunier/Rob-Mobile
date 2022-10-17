## 1.3 Map the environment


What happened when the robot move ? Why ?
>Quand le robot se déplace, le laser touche les murs et la map se crée

What happened when your robot tries to map a long corridor ?
>Le robot détecte les murs du couloir dans la limite de la range du laser. Quand le robot va longer le mur d'un long couloir, la map crée est décalée (elle n'est pas droite) par rapport au sens de la map. 

>Si on heurte un mur, on va avoir un shift de la map, un décalage et on va etre oblige de reculer et de replacer le robot en tournant sur lui meme pour qu'il corrige l'erreur.

>Quand on heurte le mur, le robot sur rviz continue d'avancer (il recoit toujours les donée d'odométrie) et au bout d'un moment, il se rend compte en comparant avec les données de capteurs laser qu'il est bloqué et se replace. Par contre cela créé un décalage sur la map, qu'il faut corriger en remappant l'espace alentour de façon clair.


## 1.4 Map files

Open the myMap.yaml and explain each lines
```yaml
image: myMap.pgm
resolution: 0.050000
origin: [-6.900000, -5.900000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

```

image:
> Path de l'image de la map

resolution:
> Résolution de la map (mètres/pixel)

origin:
> Position 2D du pixel le plus bas à gauche de la map

negate:
> Peut valoir 0 ou 1, permets d'inverser occupied_thresh et free_thresh

occupied_thresh:
> L'espace occupé sur la map (murs, robots, obstacles...)

free_thresh:
> L'espace libre sur la map (là où le robot peut se déplacer)






