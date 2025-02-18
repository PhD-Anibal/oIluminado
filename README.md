## O Iluminado

## Uso de Conversores Analógico-Digitais (ADC) no RP2040

## 📌 Descrição do Projeto

Este projeto tem como objetivo consolidar o uso do Conversor Analógico-Digital (ADC) no RP2040, utilizando um joystick para controlar:

Intensidade luminosa de LEDs RGB via PWM

Movimento de um quadrado no display OLED SSD1306

Mudança da borda do display ao pressionar o botão do joystick

Ativar ou desativar os LEDs PWM pelo botão A

## 📷 Demonstração em Vídeo

Confira a execução do código no vídeo abaixo:

🔗 Link de Youtube: [https://youtu.be/tSzL9VdP0N0](https://youtu.be/tSzL9VdP0N0)


## 🛠️ Componentes Utilizados

Raspberry Pi Pico (RP2040)

Joystick analógico (conectado aos pinos GPIO 26 e 27)

LED RGB (conectado aos GPIOs 11, 12 e 13)

Display OLED SSD1306 via I2C (GPIO 14 e 15)

Botão do joystick (GPIO 22)

Botão A (GPIO 5)

## 📡 Funcionalidades

Controle de LEDs RGB:

O LED Azul é controlado pelo eixo Y do joystick.

O LED Vermelho é controlado pelo eixo X do joystick.

Ambos os LEDs aumentam de intensidade conforme o joystick se afasta do centro (valor ADC 2048).

A variação do brilho ocorre via PWM.

Movimentação do Quadrado no Display:

O joystick também controla um quadrado de 8x8 pixels no display SSD1306.

Sua posição é ajustada proporcionalmente aos valores do ADC.

Interação com os Botões:

O botão do joystick altera a borda do display e alterna o estado do LED Verde.

O botão A ativa ou desativa os LEDs RGB PWM.

## 🛠️ Configuração do Ambiente
1. Instale o *Pico SDK* e configure no VS Code.
2. Clone este repositório:
   ```bash
   git clone https://github.com/PhD-Anibal/oIluminado.git
   

3. Compile o código, no SDK do VS code aperte em compilar segundo imagem:
   
   ![compilacao](TP_compilar.jpg)

4. Envie para a placa BitDogLab utilizando o ambiente de desenvolvimento adequado.


## 📝 Instruções de Uso

Compile e carregue o código no Raspberry Pi Pico.

Mova o joystick para ver o quadrado no display e os LEDs mudando de brilho.

Pressione o botão do joystick para alternar o estilo da borda e ligar/desligar o LED verde.

Pressione o botão A para ativar ou desativar os LEDs RGB PWM.

## 💡 Explicação Técnica

## 🎛️ Conversor ADC e Mapeamento de Valores

O ADC do RP2040 gera valores de 0 a 4095 (resolução de 12 bits).

O código converte esses valores para 0 a 64 (display) e 0 a 4095 (PWM).

## 🎨 Manipulação do Display SSD1306

Usa I2C para comunicação.

Exibe um quadrado de 8x8 pixels que se move conforme o joystick.

Alterna entre bordas diferentes ao pressionar o botão do joystick.

## 🌈 Controle de LEDs RGB

PWM é usado para ajustar a intensidade dos LEDs RGB.

A intensidade do LED Azul depende do eixo Y do joystick.

A intensidade do LED Vermelho depende do eixo X do joystick.

O LED Verde é ativado/desativado pelo botão do joystick.

## 🎛️ Uso de Interrupções (IRQ)

O código usa interrupções GPIO para capturar eventos dos botões sem precisar de polling.

Foi implementado um debounce via software para evitar leituras repetidas indevidas.
