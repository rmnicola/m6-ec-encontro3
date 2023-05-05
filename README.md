# Implementação de Filas em ROS <!-- omit in toc -->

## Conteúdo <!-- omit in toc -->
- [Implementando um subscriber em ROS](#implementando-um-subscriber-em-ros)
- [Implementando um controlador simples em ROS](#implementando-um-controlador-simples-em-ros)
- [Implementando uma fila de ações em ROS](#implementando-uma-fila-de-ações-em-ros)
- [Instalação do ambiente de simulação do turtlebot](#instalação-do-ambiente-de-simulação-do-turtlebot)
  - [Problemas comuns com o gazebo](#problemas-comuns-com-o-gazebo)

## Implementando um subscriber em ROS

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/7CkcfUkLMWQ/0.jpg)](https://www.youtube.com/watch?v=7CkcfUkLMWQ)

## Implementando um controlador simples em ROS

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/pDDB2E7CWUI/0.jpg)](https://www.youtube.com/watch?v=pDDB2E7CWUI)

## Implementando uma fila de ações em ROS

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/c9F0yM7PLKs.jpg)](https://www.youtube.com/watch?v=c9F0yM7PLKs)

## Instalação do ambiente de simulação do turtlebot

Antes de começarmos, verifique se você possui o ROS 2 instalado em seu sistema e se já configurou o seu ambiente de trabalho do ROS 2. Se não tiver feito isso, você pode seguir o guia de instalação e configuração do ROS 2 em https://docs.ros.org/en/humble/Installation.html.

A seguir, instale todos os pacotes relacionados ao turtlebot 3 utilizando o seguinte comando:
```console
sudo apt install ros-humble-turtlebot3*
```

Antes de iniciar a simulação, precisamos definir o modelo do robô a ser utilizado pelo gazebo. Para isso, utilizaremos uma variável de ambiente. Rode o comando abaixo:
```console
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
Após rodar esse comando, é necessário dar source novamente na configuração do bash. Rode:
```console
source ~/.bashrc
```
Pronto. Agora está tudo certo para rodar a simulação do turtlesim utilizando o gazebo. É possível iniciar o gazebo e depois manualmente carregar o modelo do turtlebot, mas já existe um lançador que faz isso automaticamente. Rode:
```console
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Note que o argumento `empty_world.launch.py` define qual cenário será aberto no gazebo. Tente executar o mesmo comando utilizando `turtlebot3_world.launch.py` em seu lugar.

Para mover o robô simulado basta interagir com os mesmo tópicos que existiriam no robô físico. Em específico, podemos utilizar o pacote `turtlebot3_teleop` para enviar mensagens para o tópico `cmd_vel`. Para lançar o teleop, rode:
```console
ros2 run turtlebot3_teleop teleop_keyboard
```

### Problemas comuns com o gazebo

#### O gazebo abre a interface gráfica, mas o ambiente de simulação está só com uma tela preta

Isso ocorre pois o gazebo não está conseguindo acessar o recurso de aceleração 3D. **Caso esteja utilizando o notebook do Inteli** ou qualquer outro computador com placa de vídeo integrada Intel, tente atualizar os drivers de vídeo. Se ainda assim não funcionar, a opção passa a ser desabilitar a aceleração gráfica. Rode:

```console
echo "LIBGL_ALWAYS_SOFTWARE=true" >> ~/.bashrc
```

e depois:
```console
source ~/.bashrc
```

Note que essa opção vai diminuir consideravelmente a performance do seu ambiente de simulação.
