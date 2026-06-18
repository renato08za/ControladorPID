# Controlador PID - Plataforma Didática Motor CC

Plataforma didática para controle de velocidade e posição angular de motor 
de corrente contínua (CC), desenvolvida como Trabalho de Conclusão de Curso 
na Universidade Federal do Sul e Sudeste do Pará (UNIFESSPA).

## Descrição

O sistema utiliza um Arduino integrado a uma interface gráfica em Python 
para controle e monitoramento em tempo real de um motor CC, permitindo 
ajuste dinâmico dos parâmetros dos controladores P, PI, PD e PID.

## Demonstração

[![Demonstração do Controlador PID]](https://www.youtube.com/watch?v=SEU_ID_DO_VIDEO)

## Estrutura do Repositório
ControladorPID/

├── interface_pid_controller.py   # Interface gráfica em Python

├── build.bat                     # Script para gerar o executável

└── codigos arduino/

├── pid_velocidade/

│   └── pid_velocidade.ino    # Controle de velocidade angular

└── pid_posicao/

│   └── pid_posicao.ino       # Controle de posição angular

## Requisitos

- Python 3.14+
- Arduino Uno
- Bibliotecas Python: `pyserial`, `matplotlib`, `numpy`

## Como Usar

### Interface gráfica
Execute diretamente pelo Python:
```bash
python interface_pid_controller.py
```

Ou gere o executável rodando:
```bash
build.bat
```

### Arduino
Abra o arquivo `.ino` correspondente na Arduino IDE e 
faça o upload para a placa.

## Hardware Utilizado

- Arduino Uno
- Motor CC com caixa de redução
- Encoder incremental 
- Ponte H
- Fonte de alimentação

## Autor

Renato Santos Sousa  
Bacharelado em Engenharia Elétrica — UNIFESSPA  
Orientador: Prof. Dr. Fernando de Gusmão Coutinho
