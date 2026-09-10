# Bitacora iPre Nero


- Semanas pre 31/08 - 02/09: El trabajo realizado consistio en revisar la documentacion del robot Nero, tando del manual como de los repositorios de Github, revisar la plataforma web y conexion por LAN con el brazo, finalmente la aplicacion de Mujoco para usar una version simulada del brazo, en el estado actual de esta entrada solo se puede cargar el brazo al simulador y moverlo de manera manual por medio de las configuraciones propias de Mujoco.
- Semana 31/08 - 02/09: Se armo un directorio para modificar archivos xml y crear los propios para simular el barzo en mujoco y se probo de manera directa ejecutando el xml con mujoco. Se creo la primera estructura de nodos de ros2 para comunicarse entre el usuario y el simulador.
- Semana 06 - 12 de septiembre: Se continuo con el desarrollo de la estructura de ros2 para abrir el simulador del robot nero, enfocandose principalmente en la integracion de un nodo de cinematica inversa. Realice una actualizacion del nodo de mujoco, arreglando un problema de logica interna del nodo y preparando para ahce rla conexion entre nodo simulador y cinemtaica interna.
