/*
 * Por: Anibal Maldonado
 *    Uso de conversores analógico-digitais (ADC) no RP2040
 * 
 * Este programa utiliza o display OLED SSD1306 com resolução de 128x64 pixels
 * e o microcontrolador RP2040 (Raspberry Pi Pico) para exibir informações
 * do conversor analógico-digital (ADC) e do Joystick.
 * Também mostra a leitura do botão do Joystick e do botão A.
 * 
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h" 
#include "lib/ssd1306.h"
#include "lib/font.h" 

// Definição de pinos e configurações do hardware
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22 // GPIO para botão do Joystick
#define Botao_A 5 // GPIO para botão A
#define LED_G_PIN 11  
#define LED_R_PIN 13  
#define LED_B_PIN 12  
bool bloqueia_led_RGB = false;  // Estado do LED (inicialmente permite)
bool desenha_borda = false;     // Para alterar a borda ao apertar botão do Joystick

#define DEBOUNCE_DELAY 100  // Tempo de debounce para evitar múltiplas detecções de um único toque
volatile uint32_t last_interrupt_time=0; // Armazena o tempo da última interrupção

// Função de interrupção para tratar eventos nos botões
void gpio_irq_handler(uint gpio,uint32_t events)
{   
    uint32_t current_time=to_ms_since_boot(get_absolute_time()); // Obtém o tempo atual em milissegundos
    if(current_time-last_interrupt_time>DEBOUNCE_DELAY){
      last_interrupt_time=current_time;   // Atualiza o tempo da última interrupção
      if(gpio==JOYSTICK_PB){              // caso o botão do Joystick é acionado
        desenha_borda=!desenha_borda;     // Altera borda do Display ao epertar este botão
        if(!bloqueia_led_RGB){
          gpio_put(LED_G_PIN, !gpio_get(LED_G_PIN));  // Alterna o estado do LED Verde
        }else{                           
          gpio_put(LED_G_PIN, 0);              // Garante que o LED Verde está desligado
        }
      }else{                                   // caso contrario (o botão A é acionado)
        bloqueia_led_RGB=!bloqueia_led_RGB;    // Alterna a permissão de controle dos LEDs RGB
      }
    }
}

// Inicializa e configura o PWM para um GPIO especificado
uint pwm_init_gpio(uint gpio, uint wrap) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true);  
    return slice_num;  
}

int main()
{
  stdio_init_all();

  // Configuração dos botões
  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB); 
  gpio_init(Botao_A);
  gpio_set_dir(Botao_A, GPIO_IN);
  gpio_pull_up(Botao_A);

  // Configuração das interrupções nos botões
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB,GPIO_IRQ_EDGE_FALL, true,&gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(Botao_A,GPIO_IRQ_EDGE_FALL, true,&gpio_irq_handler);

  // Configuração dos LEDs
  gpio_init(LED_B_PIN);
  gpio_set_dir(LED_B_PIN, GPIO_OUT);
  gpio_put(LED_B_PIN, false); 
  gpio_init(LED_R_PIN);
  gpio_set_dir(LED_R_PIN, GPIO_OUT);
  gpio_put(LED_R_PIN, false); 
  gpio_init(LED_G_PIN);
  gpio_set_dir(LED_G_PIN, GPIO_OUT);
  gpio_put(LED_G_PIN, false);

  // Inicialização do I2C para o display 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pono GPIO para I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pono GPIO para I2C
  gpio_pull_up(I2C_SDA);                     // Linha de dados
  gpio_pull_up(I2C_SCL);                     // Linha do clock

  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  // Configuração do ADC
  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);  
  
  uint16_t adc_value_x, adc_value_y;
  uint16_t distancia_centro_x, distancia_centro_y;

  // Inicializa PWM para os LEDs RGB
  uint pwm_wrap = 4096;   
  uint pwm_slice_b = pwm_init_gpio(LED_B_PIN, pwm_wrap); 
  uint pwm_slice_r = pwm_init_gpio(LED_R_PIN, pwm_wrap); 

  while (true)
  {
    // Leitura do ADC eixo Y e controle do LED Azul
    adc_select_input(0);
    adc_value_y = adc_read();
    distancia_centro_y=2*abs(4096/2-adc_value_y); // Multiplia por dois pois o brilho maximo é 4095
    if(!bloqueia_led_RGB){
      if(distancia_centro_y>150){   // no centro o LED RGB AZUL fica desligado
        if (distancia_centro_y > 4095) distancia_centro_y = 4095; // Limita o valor máximo
        pwm_set_gpio_level(LED_B_PIN, distancia_centro_y);
      }else{
        pwm_set_gpio_level(LED_B_PIN, 0);
      }
    }else{
      pwm_set_gpio_level(LED_B_PIN, 0);
      }
    
    adc_value_y=(64-8)-(adc_value_y*(64-8)/4096); //resolução do display 128x64 -8 de cada lado
    
    // Leitura do ADC eixo X e controle do LED Vermelho
    adc_select_input(1);
    adc_value_x = adc_read();
    distancia_centro_x=2*abs(4096/2-adc_value_x);  // Multiplia por dois pois o brilho maximo é 4095
    if(!bloqueia_led_RGB){
      if(distancia_centro_x>150){ // no centro o LED RGB VERMELHO fica desligado
        if (distancia_centro_x > 4095) distancia_centro_x = 4095; // Limita o valor máximo
        pwm_set_gpio_level(LED_R_PIN, distancia_centro_x);
        }else{
         pwm_set_gpio_level(LED_R_PIN, 0);
        }
    }else{
      pwm_set_gpio_level(LED_R_PIN, 0);
    }
    adc_value_x = (adc_value_x) * (128 - 8) / 4096;
    
    // Atualiza o conteúdo do display com animações
    ssd1306_fill(&ssd, false); // Limpa o display
    if(desenha_borda){
      ssd1306_rect(&ssd, 3, 3, 122, 60, true, false); // Desenha borda: um retângulo
    }else{
      for (int x = 3; x < 125; x += 4) {     // Desenha borda: um retângulo ponteado
        ssd1306_pixel(&ssd, x, 3, true);     // Linha superior
        ssd1306_pixel(&ssd, x, 61, true);    // Linha inferior
     }
      for (int y = 3; y < 61; y += 4) { 
        ssd1306_pixel(&ssd, 3, y, true);     // Linha esquerda
        ssd1306_pixel(&ssd, 125, y, true);   // Linha direita
      } 
    }
    
    ssd1306_rect(&ssd, adc_value_y,adc_value_x, 8, 8, true, false); // Desenha um retângulo de 8x8 que mexe com o Joystick
    ssd1306_send_data(&ssd); // Atualiza o display
    sleep_ms(500);
  }
}