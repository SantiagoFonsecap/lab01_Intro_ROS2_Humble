# lab01_Intro_ROS2_Humble
El presente laboratorio tenia como objetivo conectar nodos de ROS 2 con python haciendo uso del software Turtlesim con el fin de adaptarse al entorno de Linux y familializarse con conceptos basicos de ROS2.

Para el desarrollo del laboratorio se modifico el archivo move turtle.py creado previamente en clase, con el objetivo de que la tortuga se moviera haciendo uso de las flechas del teclado de la siguiente forma:

◦ Flecha ↑: avanzar hacia adelante.

◦ Flecha ↓: retroceder.

◦ Flecha ←: girar a la izquierda.

◦ Flecha →: girar a la derecha.

Se modifico tambien para que recibiese como entrada del teclado las inciales de los integrantes del grupo, que son J,D,M,P y S,C,F,P. 


Para conseguir los resultados mencionados, se hizo uso del modulo termiod en python con el cual se implemento la lectura de las entradas del teclado, con las cuales se especifico los patrones de movimiento deseado ante cada entrada al determinar la magnitud del movimiento lineal, su velocidad angular y duracion.
Se implementaron tambien funciones para la alineacion de la tortuga con el fin de que al dibujar las letras lo haga de la forma apropiada.

A continuacion se muestra el diagrama de flujo que representa la solucion.
![Editor _ Mermaid Chart-2025-04-27-202030](https://github.com/user-attachments/assets/43b4f338-6ca3-40a4-beca-9aa96672f5ef)


