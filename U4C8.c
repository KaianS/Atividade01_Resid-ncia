#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "ssd1306.h"

// Definições de hardware e pinos
#define I2C_PORT i2c1  // Porta I2C utilizada
#define SDA_PIN 14  // Pino SDA do barramento I2C
#define SCL_PIN 15  // Pino SCL do barramento I2C
#define OLED_ADDR 0x3C  // Endereço do display OLED

#define JOY_X 27  // Pino do eixo X do joystick
#define JOY_Y 26  // Pino do eixo Y do joystick
#define BTN_JOY 22  // Pino do botão do joystick
#define BTN_ACTION 5  // Pino do botão de ação

#define LED_R 13  // Pino do LED vermelho
#define LED_G 11  // Pino do LED verde
#define LED_B 12  // Pino do LED azul

#define DEAD_ZONE 200  // Definição da zona morta do joystick

// Estrutura global para armazenar o estado do sistema
typedef struct {
    ssd1306_t display;  // Estrutura do display OLED
    bool border_state;  // Estado da borda no display
    bool green_led_state;  // Estado do LED verde
    bool pwm_led_state;  // Estado dos LEDs PWM
    uint16_t center_x;  // Posição central do eixo X do joystick
    uint16_t center_y;  // Posição central do eixo Y do joystick
} system_state_t;

// Instância da estrutura global
system_state_t sys_state;

// Inicializa um pino GPIO como saída PWM
void pwm_init_gpio(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);  // Configura o pino para função PWM
    uint slice = pwm_gpio_to_slice_num(pin);  // Obtém o slice PWM correspondente
    pwm_set_wrap(slice, 4095);  // Define o valor máximo do PWM
    pwm_set_enabled(slice, true);  // Habilita o PWM
}

// Ajusta o valor do joystick considerando a zona morta
int16_t adjust_joystick_value(int16_t raw, int16_t center) {
    int16_t diff = raw - center;  // Calcula a diferença em relação ao centro
    return (abs(diff) < DEAD_ZONE) ? 0 : diff;  // Se dentro da zona morta, retorna 0
}

// Alterna o estado do LED verde e da borda no display
void toggle_led_border(uint pin, uint32_t events) {
    static uint32_t last_press_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Evita acionamentos múltiplos rápidos (debounce de 200ms)
    if (current_time - last_press_time < 200) {
        return;
    }
    last_press_time = current_time;

    sys_state.green_led_state = !sys_state.green_led_state;
    gpio_put(LED_G, sys_state.green_led_state);
    sys_state.border_state = !sys_state.border_state;
    printf("Botão do joystick pressionado: Borda=%d, LED Verde=%d\n", sys_state.border_state, sys_state.green_led_state);
}


// Alterna o estado dos LEDs PWM
void toggle_pwm_leds(uint pin, uint32_t events) {
    sys_state.pwm_led_state = !sys_state.pwm_led_state;  // Inverte estado do PWM
    printf("Botão de ação pressionado: PWM=%d\n", sys_state.pwm_led_state);
}

// Callback para lidar com interrupções GPIO
void gpio_callback(uint pin, uint32_t events) {
    if (pin == BTN_JOY && (events & GPIO_IRQ_EDGE_FALL)) toggle_led_border(pin, events);
    else if (pin == BTN_ACTION && (events & GPIO_IRQ_EDGE_FALL)) toggle_pwm_leds(pin, events);
}

// Realiza a calibração do joystick, determinando os valores centrais
void calibrate_joystick() {
    const int samples = 150;  // Número de amostras para calibração
    uint32_t sum_x = 0, sum_y = 0;  // Variáveis para somar os valores lidos
    for (int i = 0; i < samples; i++) {
        adc_select_input(0); sum_x += adc_read();  // Lê eixo X e soma
        adc_select_input(1); sum_y += adc_read();  // Lê eixo Y e soma
        sleep_ms(10);  // Aguarda um pequeno intervalo entre leituras
    }
    sys_state.center_x = sum_x / samples;  // Calcula média para centro X
    sys_state.center_y = sum_y / samples;  // Calcula média para centro Y
}

// Configura todo o hardware necessário
void setup_hardware() {
    stdio_init_all();  // Inicializa a comunicação serial
    adc_init();  // Inicializa o ADC
    adc_gpio_init(JOY_X);  // Configura ADC no pino X do joystick
    adc_gpio_init(JOY_Y);  // Configura ADC no pino Y do joystick

    gpio_init(BTN_JOY);  // Inicializa botão do joystick
    gpio_set_dir(BTN_JOY, GPIO_IN);  // Define como entrada
    gpio_pull_up(BTN_JOY);  // Habilita pull-up interno

    gpio_init(BTN_ACTION);  // Inicializa botão de ação
    gpio_set_dir(BTN_ACTION, GPIO_IN);  // Define como entrada
    gpio_pull_up(BTN_ACTION);  // Habilita pull-up interno

    // Configuração das interrupções nos botões
    gpio_set_irq_enabled_with_callback(BTN_JOY, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(BTN_ACTION, GPIO_IRQ_EDGE_FALL, true);

    // Inicialização dos LEDs
    pwm_init_gpio(LED_R);
    pwm_init_gpio(LED_B);
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_put(LED_G, false);

    // Inicialização do I2C e do display OLED
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN); 
    ssd1306_init(&sys_state.display, 128, 64, false, OLED_ADDR, I2C_PORT); 
    ssd1306_config(&sys_state.display); 

    sys_state.pwm_led_state = true; 
}

// Atualiza a exibição do display OLED
void update_display(uint16_t x, uint16_t y) {
    int pos_x = ((4095 - x) * 52) / 4095;  // Calcula posição X do cursor
    int pos_y = (y * 113) / 4095;  // Calcula posição Y do cursor

    ssd1306_fill(&sys_state.display, false);  // Limpa tela
    if (sys_state.border_state) {
        ssd1306_rect(&sys_state.display, 0, 0, 127, 63, 1, false);
        ssd1306_rect(&sys_state.display, 2, 2, 123, 59, 1, false);
    }
    ssd1306_draw_string(&sys_state.display, "EMBARCATECH", 20, 16);
    ssd1306_rect(&sys_state.display, pos_x, pos_y, 8, 8, 1, true);
    ssd1306_send_data(&sys_state.display);
}

// Loop principal do programa
void loop() {
    while (true) {
        adc_select_input(0);
        uint16_t x_raw = adc_read();
        adc_select_input(1);
        uint16_t y_raw = adc_read();
        printf("Joystick: X=%d, Y=%d\n", x_raw, y_raw);
        if (sys_state.pwm_led_state) {
            int16_t x_adj = adjust_joystick_value(x_raw, sys_state.center_x);
            int16_t y_adj = adjust_joystick_value(y_raw, sys_state.center_y);
            pwm_set_gpio_level(LED_R, abs(y_adj) * 2);
            pwm_set_gpio_level(LED_B, abs(x_adj) * 2);
        }
        update_display(x_raw, y_raw);
        sleep_ms(50);
    }
}

// Função principal do programa
int main() {
    setup_hardware();
    calibrate_joystick();
    loop();
    return 0;
}
