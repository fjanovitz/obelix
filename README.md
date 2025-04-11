# Obelix
Este projeto consiste em configurar e programar um robô diferencial autônomo utilizando ROS para administrar os sensores e atuadores do robô. Este robô consiste em:
- 1x Raspberry Pi 4 Model B
- 2x Motores DC 3-6V com caixa de redução
- 2x Codificadores magnéticos
- 1x Ponte H L298 N 
- 1x Laser YDLidar G2
- 1x Câmera Raspberry Pi v2 8MP
- 1x Servomotor

![image](https://github.com/user-attachments/assets/98c134a5-4b50-42f2-846e-b64b33981ecc)

Para o uso da câmera, utilizamos a biblioteca OpenCV para manipulação e reconhecimento de formas e cores.

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
