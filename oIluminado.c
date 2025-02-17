/*
 * Por: Wilton Lacerda Silva
 *    Interface Homem-Máquina (IHM) com o Display OLED
 * 
 * Este programa utiliza o display OLED SSD1306 com resolução de 128x64 pixels
 * e o microcontrolador RP2040 (Raspberry Pi Pico) para exibir informações
 * do conversor analógico-digital (ADC) e do Joystick.
 * Também mostra a leitura dos botões do Joystick e do botão A.
 * 
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h" 

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

#define DEBOUNCE_DELAY 100  // Tempo de debounce para evitar múltiplas detecções de um único toque
volatile uint32_t last_interrupt_time=0; // Armazena o tempo da última interrupção

// Função de interrupção para tratar eventos nos botões
void gpio_irq_handler(uint gpio,uint32_t events)
{
    uint16_t i;
    uint32_t valor_led;
        
    uint32_t current_time=to_ms_since_boot(get_absolute_time()); // Obtém o tempo atual em milissegundos
    if(current_time-last_interrupt_time>DEBOUNCE_DELAY){
      if(gpio==JOYSTICK_PB){

        last_interrupt_time=current_time; // Atualiza o tempo da última interrupção
        if(!bloqueia_led_RGB){
          gpio_put(LED_G_PIN, !gpio_get(LED_G_PIN));
        }else{
          gpio_put(LED_G_PIN, 0);
        }
      }else{
        bloqueia_led_RGB=!bloqueia_led_RGB;
      }


        
    }
}

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

  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB); 


  gpio_init(Botao_A);
  gpio_set_dir(Botao_A, GPIO_IN);
  gpio_pull_up(Botao_A);

  // Configuração das interrupções nos botões
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB,GPIO_IRQ_EDGE_FALL, true,&gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(Botao_A,GPIO_IRQ_EDGE_FALL, true,&gpio_irq_handler);

    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_put(LED_B_PIN, false); 
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_put(LED_R_PIN, false); 
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_put(LED_G_PIN, false);

  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);  
  

  uint16_t adc_value_x;
  uint16_t adc_value_y;  
  uint16_t distancia_centro_x;
  uint16_t distancia_centro_y;

  char str_x[5];  // Buffer para armazenar a string
  char str_y[5];  // Buffer para armazenar a string  
 uint pwm_wrap = 4096;   
uint pwm_slice_b = pwm_init_gpio(LED_B_PIN, pwm_wrap); 
uint pwm_slice_r = pwm_init_gpio(LED_R_PIN, pwm_wrap); 

  bool cor = true;
  while (true)
  {
    adc_select_input(0); // Seleciona o ADC para eixo Y. O pino 26 como entrada analógica
    adc_value_y = adc_read();
    distancia_centro_y=abs(4096/2-adc_value_y);
    if(!bloqueia_led_RGB){
      if(distancia_centro_y>50){
        pwm_set_gpio_level(LED_B_PIN, distancia_centro_y);
      }else{
        pwm_set_gpio_level(LED_B_PIN, 0);
      }
    }else{
      pwm_set_gpio_level(LED_B_PIN, 0);
      }
    
    adc_value_y=(64-8)-(adc_value_y*(64-8)/4096); //resolução do display 128x64 -8 de cada lado
    
    adc_select_input(1); // Seleciona o ADC para eixo X. O pino 27 como entrada analógica
    adc_value_x = adc_read();
    distancia_centro_x=abs(4096/2-adc_value_x);
    if(!bloqueia_led_RGB){
      if(distancia_centro_x>50){
        pwm_set_gpio_level(LED_R_PIN, distancia_centro_x);
        }else{
         pwm_set_gpio_level(LED_R_PIN, 0);
        }
    }else{
      pwm_set_gpio_level(LED_R_PIN, 0);
    }
    adc_value_x = (adc_value_x) * (128 - 8) / 4096;
    
    //cor = !cor;
    // Atualiza o conteúdo do display com animações
    ssd1306_fill(&ssd, !cor); // Limpa o display
    
    if(gpio_get(LED_G_PIN)){
      ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Desenha um retângulo
    }else{
      for (int x = 3; x < 125; x += 4) { 
        ssd1306_pixel(&ssd, x, 3, cor);     // Linha superior
        ssd1306_pixel(&ssd, x, 61, cor);    // Linha inferior
     }
      for (int y = 3; y < 61; y += 4) { 
         ssd1306_pixel(&ssd, 3, y, cor);     // Linha esquerda
        ssd1306_pixel(&ssd, 125, y, cor);   // Linha direita
      } 
    }
    
    ssd1306_rect(&ssd, adc_value_y,adc_value_x, 8, 8, cor, !cor); // Desenha um retângul       
    ssd1306_send_data(&ssd); // Atualiza o display
    sleep_ms(500);
  }
}