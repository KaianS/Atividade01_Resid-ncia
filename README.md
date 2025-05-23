﻿# BorderCounter: Sistema de Contagem por Interação Periférica

Este projeto implementa um sistema interativo que permite ao usuário controlar um cursor em uma interface visual e incrementar um contador através do toque nas bordas do display. Utilizando a placa BitDogLab baseada no microcontrolador RP2040, o sistema registra interações com as bordas do display, exibe o valor atual do contador na matriz de LEDs e fornece feedback visual e sonoro, criando uma experiência multissensorial completa.

## Objetivos do Projeto

- Compreender o funcionamento do conversor analógico-digital (ADC) no RP2040.
- Utilizar PWM para controlar a intensidade dos LEDs RGB com base nos valores do joystick.
- Representar a posição do joystick em um display SSD1306 por meio de um quadrado móvel.
- Aplicar o protocolo de comunicação I2C para integração com o display.
- Controlar uma matriz de LEDs WS2812B usando PIO para exibir dígitos numéricos.
- Implementar um contador de toques na borda do display.

## Funcionalidades

### LEDs RGB
- **LED Azul**: Controlado pelo eixo Y do joystick. O brilho aumenta conforme o movimento do joystick para cima ou para baixo.
- **LED Vermelho**: Controlado pelo eixo X do joystick. O brilho aumenta conforme o movimento do joystick para a esquerda ou direita.
- Os LEDs são controlados via PWM, permitindo variação suave de intensidade.

### Display SSD1306
- Um quadrado de 8x8 pixels será exibido no display e se moverá proporcionalmente aos valores capturados pelo joystick.
- O display mostra o estado atual da matriz de LEDs e o valor do contador.

### Matriz de LEDs WS2812B
- Exibe um número (0-9) correspondente ao valor atual do contador.
- O contador aumenta cada vez que o cursor toca na borda do display.
- O número permanece fixo na matriz de LEDs, independentemente da movimentação do joystick.

### Botões e Controles
- **Botão do Joystick**: Alterna o estado do LED verde, a borda do display e ativa/desativa a matriz de LEDs WS2812B.
- **Botão de Ação**: Reseta o contador para zero e alterna o estado dos LEDs PWM.
- **Tocar na Borda**: Cada vez que o cursor toca na borda do display, o contador é incrementado e o número correspondente é exibido na matriz de LEDs.

### Feedback Sonoro
- Um buzzer fornece feedback sonoro para várias ações, como pressionar botões e tocar na borda do display.
- O tom varia de acordo com a posição do joystick e o valor do contador.

## Requisitos de Hardware

- **Placa BitDogLab** com:
  - LED RGB conectado aos pinos GPIO 11, 12 e 13.
  - Botão do Joystick na GPIO 22.
  - Joystick nos pinos GPIO 26 e 27.
  - Botão A na GPIO 5.
  - Display SSD1306 via I2C (GPIO 14 e GPIO 15).
  - Matriz de LEDs WS2812B (5x5) no pino GPIO 7.
  - Buzzer no pino GPIO 10.

## Requisitos do Projeto

1. **Uso de interrupções**: Todas as funcionalidades dos botões devem ser implementadas usando interrupções (IRQ).
2. **Debouncing**: Implementação do tratamento de debounce dos botões via software.
3. **Uso do Display 128x64**: Implementação do protocolo I2C para controle do display e visualização do quadrado.
4. **Matriz de LEDs WS2812B**: Implementação do controle da matriz usando PIO para exibir dígitos numéricos.
5. **Contador de Bordas**: Implementação de um contador que incrementa quando o cursor toca nas bordas do display.
6. **Organização do código**: O código deve ser bem estruturado, comentado e fácil de entender.

## Instruções de Uso

1. Clone o repositório para sua máquina:
    ```bash
    https://github.com/KaianS/Atividade01_Resid-ncia.git
    ```
2. Conecte a placa BitDogLab.
3. Compile e carregue o código no seu RP2040.
4. Execute o código e:
   - Movimente o joystick para controlar o cursor no display e os LEDs RGB.
   - Leve o cursor até a borda do display para incrementar o contador.
   - Pressione o botão do joystick para alternar a exibição da matriz de LEDs.
   - Pressione o botão de ação para resetar o contador.

## Como Contribuir

1. Faça um fork do repositório.
2. Crie uma branch para sua feature (`git checkout -b minha-feature`).
3. Faça commit das suas alterações (`git commit -am 'Adiciona nova feature'`).
4. Envie a branch para o repositório remoto (`git push origin minha-feature`).
5. Abra um pull request.

## Vídeo de Demonstração

[![Watch the video](https://img.youtube.com/vi/aqNkw2MdNyg/maxresdefault.jpg)](https://youtu.be/aqNkw2MdNyg)

### [Watch this video on YouTube](https://youtu.be/aqNkw2MdNyg)

## Considerações Finais

Este projeto serve como uma excelente oportunidade para consolidar conhecimentos sobre programação de microcontroladores, controle de LEDs via PWM, manipulação de hardware, uso do protocolo I2C e controle de LEDs WS2812B com PIO. A integração entre diferentes periféricos demonstra o potencial do RP2040 para aplicações embarcadas complexas.
