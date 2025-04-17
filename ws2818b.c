#include "ws2818b.h"
#include "ws2818b.pio.h"

// MACRO
#define LED_PIN 7
#define LED_COUNT 25

// Matriz virtual 5x5
#define MATRIX_SIZE 5

PIO np_pio;
uint sm;
struct pixel_t {
    uint8_t G, R, B;
};
typedef struct pixel_t npLED_t;
npLED_t leds[LED_COUNT];

void init_leds(void) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, true);
    ws2818b_program_init(np_pio, sm, offset, LED_PIN, 800000.f);
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i].R = leds[i].G = leds[i].B = 0;
    }
}

// Limpa os LEDs
void clear_leds(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i].R = leds[i].G = leds[i].B = 0;
    }
}

// Define a cor de um LED
void set_led(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < LED_COUNT) {
        leds[index].R = r;
        leds[index].G = g;
        leds[index].B = b;
    }
}

// Define a cor de todos os LEDs
void set_all_leds(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i].R = r;
        leds[i].G = g;
        leds[i].B = b;
    }
}

// Escreve os LEDs
void write_leds(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
}

// Converte coordenadas X,Y para índice na matriz 5x5
int xy_to_index(int x, int y) {
    // Garante que estamos dentro dos limites da matriz
    if (x < 0) x = 0;
    if (x >= MATRIX_SIZE) x = MATRIX_SIZE - 1;
    if (y < 0) y = 0;
    if (y >= MATRIX_SIZE) y = MATRIX_SIZE - 1;
    
    // Converte coordenadas 2D para índice linear
    return y * MATRIX_SIZE + x;
}

// Exibe um pixel na posição correspondente ao joystick
void display_joystick_position(int x_pos, int y_pos, uint8_t r, uint8_t g, uint8_t b) {
    clear_leds();
    
    // Mapeia valores x_pos e y_pos para coordenadas na matriz 5x5
    int matrix_x = (x_pos * (MATRIX_SIZE - 1)) / 4095;
    int matrix_y = (y_pos * (MATRIX_SIZE - 1)) / 4095;
    
    // Define o LED na posição do joystick
    int led_index = xy_to_index(matrix_x, matrix_y);
    set_led(led_index, r, g, b);
    
    write_leds();
}

// Exibe um padrão predefinido na matriz de LEDs
void display_pattern(uint8_t pattern) {
    clear_leds();
    
    switch (pattern) {
        case 0:
            // Padrão de "X"
            set_led(0, 50, 0, 0);      // Canto superior esquerdo
            set_led(4, 50, 0, 0);      // Canto superior direito
            set_led(6, 50, 0, 0);      // Meio
            set_led(12, 50, 0, 0);     // Centro
            set_led(18, 50, 0, 0);     // Meio
            set_led(20, 50, 0, 0);     // Canto inferior esquerdo
            set_led(24, 50, 0, 0);     // Canto inferior direito
            break;
        case 1:
            // Padrão de "+"
            set_led(2, 0, 50, 0);      // Topo
            set_led(7, 0, 50, 0);      // Meio superior
            set_led(10, 0, 50, 0);     // Esquerda centro
            set_led(11, 0, 50, 0);     // Meio esquerdo
            set_led(12, 0, 50, 0);     // Centro
            set_led(13, 0, 50, 0);     // Meio direito
            set_led(14, 0, 50, 0);     // Direita centro
            set_led(17, 0, 50, 0);     // Meio inferior
            set_led(22, 0, 50, 0);     // Base
            break;
        case 2:
            // Padrão de "O"
            set_led(1, 0, 0, 50);      // Topo
            set_led(2, 0, 0, 50);      // Topo
            set_led(3, 0, 0, 50);      // Topo
            set_led(5, 0, 0, 50);      // Esquerda
            set_led(9, 0, 0, 50);      // Esquerda
            set_led(10, 0, 0, 50);     // Topo-esquerda
            set_led(14, 0, 0, 50);     // Topo-direita
            set_led(15, 0, 0, 50);     // Direita
            set_led(19, 0, 0, 50);     // Direita
            set_led(21, 0, 0, 50);     // Base
            set_led(22, 0, 0, 50);     // Base
            set_led(23, 0, 0, 50);     // Base
            break;
        case 3:
            // Padrão de seta para cima
            for (int i = 20; i < 25; i++) {
                set_led(i, 50, 50, 0);  // Base da seta (linha inferior)
            }
            set_led(12, 50, 50, 0);     // Centro da seta
            set_led(7, 50, 50, 0);      // Subindo
            set_led(2, 50, 50, 0);      // Topo da seta
            set_led(6, 50, 50, 0);      // Lateral esquerda
            set_led(8, 50, 50, 0);      // Lateral direita
            break;
        default:
            // Padrao padrão: borda
            for (int i = 0; i < MATRIX_SIZE; i++) {
                // Bordas superior e inferior
                set_led(i, 15, 15, 15);
                set_led(i + (MATRIX_SIZE - 1) * MATRIX_SIZE, 15, 15, 15);
                
                // Bordas laterais
                set_led(i * MATRIX_SIZE, 15, 15, 15);
                set_led(i * MATRIX_SIZE + (MATRIX_SIZE - 1), 15, 15, 15);
            }
            break;
    }
    
    write_leds();
}

// Exibe um número de 0 a 9 na matriz de LEDs
void display_number(int number) {
    clear_leds();
    
    // Certifique-se de que number está entre 0 e 9
    number = number % 10;
    
    switch (number) {
        case 0:
            // Define os LEDs acesos para exibir o número 0
            set_led(1, 50, 0, 0);
            set_led(2, 50, 0, 0);
            set_led(3, 50, 0, 0);
            set_led(5, 50, 0, 0);
            set_led(9, 50, 0, 0);
            set_led(10, 50, 0, 0);
            set_led(14, 50, 0, 0);
            set_led(15, 50, 0, 0);
            set_led(19, 50, 0, 0);
            set_led(21, 50, 0, 0);
            set_led(22, 50, 0, 0);
            set_led(23, 50, 0, 0);
            break;
        case 1:
            // Define os LEDs acesos para exibir o número 1
            set_led(1, 0, 50, 0);
            set_led(2, 0, 50, 0);
            set_led(3, 0, 50, 0);
            set_led(7, 0, 50, 0);
            set_led(12, 0, 50, 0);
            set_led(17, 0, 50, 0);
            set_led(22, 0, 50, 0);
            set_led(16, 0, 50, 0);
            break;
        case 2:
            // Define os LEDs acesos para exibir o número 2
            set_led(0, 0, 0,  15);
            set_led(1, 0, 0,  15);
            set_led(2, 0, 0,  15);
            set_led(3, 0, 0,  15);
            set_led(4, 0, 0,  15);
            set_led(5, 0, 0,  15);
            set_led(10, 0, 0, 15);
            set_led(11, 0, 0, 15);
            set_led(12, 0, 0, 15);
            set_led(13, 0, 0, 15);
            set_led(14, 0, 0, 15);
            set_led(19, 0, 0, 15);
            set_led(20, 0, 0, 15);
            set_led(21, 0, 0, 15);
            set_led(22, 0, 0, 15);
            set_led(23, 0, 0, 15);
            set_led(24, 0, 0, 15);
            break;
        case 3:
            // Define os LEDs acesos para exibir o número 3
            set_led(0, 15, 50, 0);
            set_led(1, 15, 50, 0);
            set_led(2, 15, 50, 0);
            set_led(3, 15, 50, 0);
            set_led(4, 15, 50, 0);
            set_led(9, 15, 50, 0);
            set_led(10, 15, 50, 0);
            set_led(11, 15, 50, 0);
            set_led(12, 15, 50, 0);
            set_led(13, 15, 50, 0);
            set_led(19, 15, 50, 0);
            set_led(20, 15, 50, 0);
            set_led(21, 15, 50, 0);
            set_led(22, 15, 50, 0);
            set_led(23, 15, 50, 0);
            set_led(24, 15, 50, 0);
            break;
        case 4:
            // Define os LEDs acesos para exibir o número 4
            set_led(1, 15, 15, 0);
            set_led(8, 15, 15, 0);
            set_led(11, 15, 15, 0);
            set_led(18, 15, 15, 0);
            set_led(21, 15, 15, 0);
            set_led(12, 15, 15, 0);
            set_led(13, 15, 15, 0);
            set_led(16, 15, 15, 0);
            set_led(23, 15, 15, 0);
            break;
        case 5:
            // Define os LEDs acesos para exibir o número 5
            set_led(0, 0, 0,  15);
            set_led(1, 0, 0,  15);
            set_led(2, 0, 0,  15);
            set_led(3, 0, 0,  15);
            set_led(4, 0, 0,  15);
            set_led(9, 0, 0,  15);
            set_led(10, 0, 0, 15);
            set_led(11, 0, 0, 15);
            set_led(12, 0, 0, 15);
            set_led(13, 0, 0, 15);
            set_led(14, 0, 0, 15);
            set_led(15, 0, 0, 15);
            set_led(20, 0, 0, 15);
            set_led(21, 0, 0, 15);
            set_led(22, 0, 0, 15);
            set_led(23, 0, 0, 15);
            set_led(24, 0, 0, 15);
            break;
        case 6:
            // Define os LEDs acesos para exibir o número 6
            set_led(0, 0, 50, 15);
            set_led(1, 0, 50, 15);
            set_led(2, 0, 50, 15);
            set_led(3, 0, 50, 15);
            set_led(4, 0, 50, 15);
            set_led(5, 0, 50, 15);
            set_led(9, 0, 50, 15);
            set_led(10, 0, 50, 15);
            set_led(11, 0, 50, 15);
            set_led(12, 0, 50, 15);
            set_led(13, 0, 50, 15);
            set_led(14, 0, 50, 15);
            set_led(15, 0, 50, 15);
            set_led(20, 0, 50, 15);
            set_led(21, 0, 50, 15);
            set_led(22, 0, 50, 15);
            set_led(23, 0, 50, 15);
            set_led(24, 0, 50, 15);
            break;
        case 7:
            // Define os LEDs acesos para exibir o número 7
            set_led(20, 10, 0, 10);
            set_led(21, 10, 0, 10);
            set_led(22, 10, 0, 10);
            set_led(23, 10, 0, 10);
            set_led(24, 10, 0, 10);
            set_led(18, 10, 0, 10);
            set_led(12, 10, 0, 10);
            set_led(6, 10, 0, 10);
            set_led(4, 10, 0, 10);
            break;
        case 8:
            // Define os LEDs acesos para exibir o número 8
            set_led(24, 0, 10, 10);
            set_led(23, 0, 10, 10);
            set_led(22, 0, 10, 10);
            set_led(21, 0, 10, 10);
            set_led(20, 0, 10, 10);
            set_led(19, 0, 10, 10);
            set_led(15, 0, 10, 10);
            set_led(14, 0, 10, 10);
            set_led(13, 0, 10, 10);
            set_led(12, 0, 10, 10);
            set_led(11, 0, 10, 10);
            set_led(10, 0, 10, 10);
            set_led(9, 0, 10, 10);
            set_led(5, 0, 10, 10);
            set_led(4, 0, 10, 10);
            set_led(3, 0, 10, 10);
            set_led(2, 0, 10, 10);
            set_led(1, 0, 10, 10);
            set_led(0, 0, 10, 10);
            break;
        case 9:
            // Define os LEDs acesos para exibir o número 9
            set_led(24, 0, 10, 10);
            set_led(23, 0, 10, 10);
            set_led(22, 0, 10, 10);
            set_led(21, 0, 10, 10);
            set_led(20, 0, 10, 10);
            set_led(19, 0, 10, 10);
            set_led(15, 0, 10, 10);
            set_led(14, 0, 10, 10);
            set_led(13, 0, 10, 10);
            set_led(12, 0, 10, 10);
            set_led(11, 0, 10, 10);
            set_led(10, 0, 10, 10);
            set_led(9, 0, 10, 10);
            set_led(4, 0, 10, 10);
            set_led(3, 0, 10, 10);
            set_led(2, 0, 10, 10);
            set_led(1, 0, 10, 10);
            set_led(0, 0, 10, 10);
            break;
        default:
            break;
    }
    write_leds();
} 