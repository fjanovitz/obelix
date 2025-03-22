# obelix
Projeto da disciplina IF826 (Robótica) para configuração e controle do robô Obelix utilizando ROS Noetic.

## Como rodar o projeto

No diretório obelix, dê os seguintes comandos:

```bash
catkin_make
```
e 

```bash
source devel/setup.bash
```

Depois crie 3 terminais, em um rode:

```bash
roscore
```

Usando o pacote da camera de exemplo, no segundo terminal rode:

```bash
rosrun camera camera_server.py
```

E no terceiro:

```bash
rosrun camera camera_client.py
```