# lab01_Intro_ROS2_Humble
El presente laboratorio tenía como objetivo conectar nodos de ROS 2 con Python, haciendo uso del software Turtlesim, con el fin de adaptarse al entorno de Linux y familiarizarse con conceptos básicos de ROS 2.

Para el desarrollo del laboratorio, se modificó el archivo move_turtle.py creado previamente en clase, con el objetivo de que la tortuga se moviera haciendo uso de las flechas del teclado de la siguiente forma:

◦ Flecha ↑: avanzar hacia adelante.

◦ Flecha ↓: retroceder.

◦ Flecha ←: girar a la izquierda.

◦ Flecha →: girar a la derecha.

Se modificó también para que recibiera como entrada del teclado las iniciales de los integrantes del grupo, que son: J, D, M, P y S, C, F, P.

Para conseguir los resultados mencionados, se hizo uso del módulo termios en Python, con el cual se implementó la lectura de las entradas del teclado. Con estas entradas se especificaron los patrones de movimiento deseados, determinando la magnitud del movimiento lineal, su velocidad angular y su duración.

Se implementaron también funciones para la alineación de la tortuga, con el fin de que, al dibujar las letras, lo hiciera de la forma apropiada.

A continuación, se muestra el diagrama de flujo que representa la solución:
![Editor _ Mermaid Chart-2025-04-27-202030](https://github.com/user-attachments/assets/43b4f338-6ca3-40a4-beca-9aa96672f5ef)

El código recibe la entrada del teclado y determina si es una letra de las iniciales del nombre de los miembros del equipo o si es una flecha, y ejecuta la acción asociada. En caso de ser una letra, la tortuga se posicionará en el ángulo y sentido adecuados para dibujar la letra deseada. Una vez que la acción se completa, el código permanece a la espera de otra entrada para su ejecución.
