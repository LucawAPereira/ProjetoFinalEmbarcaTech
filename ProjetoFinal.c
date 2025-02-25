
/* 
    Código desenvolvido por Lucas Almeida Pereira - tic370100367

    Este software tem como objetivo monitorar os níveis de reservatórios de água e executar
    ações específicas conforme os dados recebidos de um sensor ultrassônico, que neste projeto é
    simulado pelo joystick.

    Funcionalidades principais:
      - Leitura analógica do "Joystick" para simulação do sensor ultrassônico.
      - Conversão dos valores lidos do Joystick.
      - Exibição dos níveis analógicos em tempo real no display OLED (gráfica e percentual).
      - Controle e interação via botões da placa BitDogLab para ativação/desativação de funções.
      - Emissão de alertas sonoros através de buzzers para níveis críticos (abaixo de 10% e acima de 100%).
      - "Botao A" ativa/desativa a exibição dos níveis em tempo real na matriz de LEDs 5x5 RGB.
      - "Botao B" ativa/desativa os motores automáticos (enchimento e drenagem), simbolizados pelos Leds vermelho e verde, além de ativar/desativar os buzzers
      - Acionamento automático dos motores:
            * Motor de enchimento: ativado quando o nível está abaixo de 10%. (Led VERMELHO)
            * Motor de drenagem: ativado quando o nível ultrapassa 100%. (Led VERDE)

    O software demonstra uma solução prática e de baixo custo para o monitoramento e controle
    inteligente de reservatórios de água, integrando tecnologias de sistemas embarcados e IoT.
*/



#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "ws2812.pio.h"

#define buzzer1 21 //Pino do buzzer 1
#define buzzer2 10 //Pino do buzzer 2
#define VRY 27 // potenciomentro eixo y conectada a placa BitDogLab
#define BotaoA 5 // botao A da placa BitDogLab
#define BotaoB 6 // botao B da placa BitDogLab
#define BotaoJY 22 // pino do botao joystick
#define LEDr 13 // pino led rgb vermelho
#define LEDg 11 // pino led rgb azul
#define div 10    // referente ao pwm
#define wrap 1000 // referente ao pwm
#define endereco 0x3C // referente ao display OLED
#define I2C_PORT i2c1 // refente ao display OLED
#define I2C_SDA 14 // pino scl rp2040
#define I2C_SCL 15 // pino scl rp2040
#define IS_RGBW false
#define NUM_PIXELS 25
#define WS2812_PIN 7

uint slice_r; // slice referente a porta conectada ao led rgb vermelho
uint slice_b; // slice referente a porta conectada ao led rgb azul

bool quadradinho_pintado1=false;
bool quadradinho_pintado2=false;
bool cor=true; // referente ao display OLED
bool ativacao_buzzer=true; // referente ao botaoB ativar/desativar o buzzer
bool ativar_matriz=false; // referente a ativacao da matriz 5x5
bool boasvindas=true;
static volatile uint32_t last_time = 0;
int conversao_porcentagem_y=0;


void play_note(int frequency, int duration, int pin); //funcao que ativa os buzzers
void alerta_nivel_baixo(); // funcao buzzer alerta nivel baixo
void alerta_nivel_alto(); // funcao buzzer alerta nivel alto
void gpio_irq_handler(uint gpio, uint32_t events); // funcao para interrupcao
void draw_composite_led_matrix(int nivel);
void set_matrix_leds(uint32_t colors[NUM_PIXELS]);
void clear_matrix();


bool led_buffer0[NUM_PIXELS] = { // desenho da caixa lado de fora (bordas)
    1, 1, 1, 1, 1, 
    1, 0, 0, 0, 1,                      // [0,1,2,3,4,
    1, 0, 0, 0, 1,                      //  5,6,7,8,9
    1, 0, 0, 0, 1,                      //  10,11,12,13,14   ---> como realmente é a ordem da matriz de leds
    1, 0, 0, 0, 1                       //  15,16,17,18,19
};                                      //  20,21,22,23,24]

bool led_buffer1[NUM_PIXELS] = { // primeiro estagio (vazio - vermelho) 25%
    0, 0, 0, 0, 0, 
    0, 1, 1, 1, 0,                    
    0, 0, 0, 0, 0,                      
    0, 0, 0, 0, 0,                     
    0, 0, 0, 0, 0                       
}; 

bool led_buffer2[NUM_PIXELS] = { // segundo estagio (violeta) 50%
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0,                    
    0, 1, 1, 1, 0,                      
    0, 0, 0, 0, 0,                     
    0, 0, 0, 0, 0                       
};                                      

bool led_buffer3[NUM_PIXELS] = { // terceiro estagio (azul) 75%
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0,                    
    0, 0, 0, 0, 0,                      
    0, 1, 1, 1, 0,                     
    0, 0, 0, 0, 0                       
}; 

bool led_buffer4[NUM_PIXELS] = { // ultimo estagio (verde - cheio) 100%
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0,                    
    0, 0, 0, 0, 0,                      
    0, 0, 0, 0, 0,                     
    0, 1, 1, 1, 0                       
}; 


static inline void put_pixel(uint32_t pixel_grb)  // protocolo WS18
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) //  protoclo WS2812
{
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}


int main()
{
    stdio_init_all();
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    gpio_set_function(LEDr, GPIO_FUNC_PWM);
    gpio_set_function(LEDg, GPIO_FUNC_PWM);

    slice_r = pwm_gpio_to_slice_num(LEDr);
    slice_b = pwm_gpio_to_slice_num(LEDg);

    pwm_set_clkdiv(slice_r, div);
    pwm_set_wrap(slice_r, wrap);
    pwm_set_enabled(slice_r, true);

    pwm_set_clkdiv(slice_b, div);
    pwm_set_wrap(slice_b, wrap);
    pwm_set_enabled(slice_b, true);

    gpio_init(buzzer1);
    gpio_set_dir(buzzer1, GPIO_OUT);
    gpio_init(buzzer2);
    gpio_set_dir(buzzer2, GPIO_OUT);

    gpio_init(BotaoA);
    gpio_set_dir(BotaoA, GPIO_IN);
    gpio_pull_up(BotaoA);
    gpio_init(BotaoB);
    gpio_set_dir(BotaoB, GPIO_IN);
    gpio_pull_up(BotaoB);
    gpio_init(BotaoJY);
    gpio_set_dir(BotaoJY, GPIO_IN);
    gpio_pull_up(BotaoJY);

    adc_init();
    adc_gpio_init(VRY);

    gpio_set_irq_enabled_with_callback(BotaoA ,GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // interrupcao para o BotaoA
    gpio_set_irq_enabled_with_callback(BotaoB ,GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // interrupcao para o BotaoB

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_t ssd; // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd);


    while (true)
    {
        ssd1306_fill(&ssd, !cor); // Limpa o display

        adc_select_input(0);
        uint16_t VRY_value = adc_read();
        conversao_porcentagem_y = (VRY_value*100)/3560; // escolhido o nivel "3560" para corrigir a leitura dos 50% joystick
        int conversao_nivel_y = VRY_value*90/3560; // conversao para preenchimento simulando um nivel de agua
        char texto[4]; // Vetor para armazenar o caractere, necessario para desenhar para dentro do display
        sprintf(texto, "%d",conversao_porcentagem_y); // converte para caractere

        if(boasvindas) // tela inicial para boas vindas embarcaTech
        {
    const uint8_t bitmap_128x64[] = {             // EmbarcaTech logo inicial
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0x73, 0xee, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0x03, 0xf8, 0xff, 0xff, 
	0x0f, 0xe0, 0x3f, 0xfe, 0x03, 0xf8, 0xff, 0xff, 0xff, 0xe1, 0x1f, 0xfc, 0x83, 0xff, 0xff, 0xff, 
	0xff, 0xe1, 0x3f, 0xfc, 0x83, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0x7f, 0xfe, 0x03, 0xf0, 0xff, 0xff, 
	0x0f, 0xe0, 0x3f, 0xfe, 0x03, 0xf8, 0xff, 0xff, 0xff, 0xe1, 0x07, 0xe0, 0x83, 0xff, 0xff, 0xff, 
	0xff, 0xe1, 0x03, 0xc0, 0x83, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0x71, 0x86, 0x03, 0xf0, 0xff, 0xff, 
	0x1f, 0xe0, 0x79, 0x8e, 0x03, 0xf8, 0xff, 0xff, 0xff, 0xe1, 0x79, 0x8e, 0x83, 0xff, 0xff, 0xff, 
	0xff, 0xe1, 0x71, 0x8e, 0x83, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0x61, 0x8e, 0x03, 0xf0, 0xff, 0xff, 
	0xff, 0xe1, 0x03, 0xc6, 0x83, 0xff, 0xff, 0xff, 0xff, 0xe1, 0x07, 0xee, 0x83, 0xff, 0xff, 0xff, 
	0x1f, 0xe0, 0xff, 0xff, 0x03, 0xf8, 0xff, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0x03, 0xf0, 0xff, 0xff, 
	0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0x1f, 0x73, 0xc6, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x73, 0xe6, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x3f, 0x00, 0x00, 0xfc, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xfb, 0xff, 0xfd, 0xff, 0xff, 0xe3, 0xff, 0xff, 0xfb, 0xff, 0xfb, 
	0xff, 0xff, 0xf1, 0xff, 0xff, 0xfb, 0xff, 0xfb, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xff, 0x1f, 0x00, 0xff, 
	0xff, 0xff, 0x07, 0x00, 0xff, 0xef, 0xfb, 0xfe, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xf7, 0xfb, 0xfd, 
	0xff, 0xff, 0xf9, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 
	0xff, 0xff, 0xf0, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xf3, 0xfb, 0xfb, 0xff, 0xff, 0x03, 0x00, 0xff, 0xe7, 0xfb, 0xfd, 
	0xff, 0xff, 0x0f, 0x00, 0xff, 0x8f, 0x7b, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xf8, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0x07, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0x07, 0x00, 0x00, 0xff, 0x1f, 0x00, 0xff, 0xff, 0xff, 0xf1, 0x0f, 0xff, 0xe7, 0xff, 0xfc, 
	0xff, 0xff, 0xf8, 0x1f, 0xff, 0xf3, 0xff, 0xfd, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xfb, 0xff, 0xfb, 
	0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfb, 0xff, 0xfb, 0xff, 0xff, 0xe1, 0x0f, 0xff, 0xfb, 0xff, 0xfb, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xfb, 0xff, 0xfb, 0xff, 0xff, 0x03, 0x80, 0xff, 0xf3, 0xff, 0xf9, 
	0xff, 0xff, 0x0f, 0xe0, 0xff, 0xe7, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0x3f, 0xfe, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xe3, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x61, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x61, 0x1c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x70, 0x3e, 0x07, 0x00, 0x00, 0xf8, 
	0xff, 0xff, 0x78, 0x3e, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xff, 0x78, 0x3e, 0xff, 0xf3, 0xff, 0xff, 
	0xff, 0xff, 0x70, 0x1e, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0x61, 0x86, 0xff, 0xfb, 0xff, 0xff, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0xff, 0xfb, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0xf8, 0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x7f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x03, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf1, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf1, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x87, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xc3, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xe7, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xe7, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x61, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x71, 0x1c, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x78, 0x3e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x78, 0x3e, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x78, 0x1e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x71, 0x8e, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x0f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

    ssd1306_t ssd_bm;
    ssd1306_init_bm(&ssd_bm, 128, 64, false, 0x3C, i2c1); 
    ssd1306_config(&ssd_bm);
    ssd1306_draw_bitmap(&ssd_bm, bitmap_128x64); // comeca a desenhar a logo

    sleep_ms(5000);
    ssd1306_draw_string(&ssd, "Desenvolvido", 0, 0);
    ssd1306_draw_string(&ssd, "porJ", 90, 10); // "j" representa o caracter ":"
    ssd1306_draw_string(&ssd, "Lucas A Pereira ", 0, 40);
    ssd1306_send_data(&ssd);
    sleep_ms(3000);
    boasvindas=false;
        }
        
        if(conversao_porcentagem_y > 100){ //if, haver com a posicao do caractere da porcentagem "%"
            if (ativar_matriz){
                draw_composite_led_matrix(conversao_porcentagem_y);
            }
            ssd1306_fill_region(&ssd, 1, 1, 38, 61, conversao_nivel_y);
            ssd1306_draw_string(&ssd, "NIVEL: ", 64, 2);
            ssd1306_draw_string(&ssd, "B1", 85, 39); 
            ssd1306_draw_string(&ssd, "B2", 85, 53); 
            ssd1306_rect(&ssd, 0, 0, 40, 63,cor,!cor);
            ssd1306_rect(&ssd, 103, 70, 8, 8, 1,quadradinho_pintado1); // quadradinho menor
            ssd1306_rect(&ssd, 117, 70, 8, 8, 1,quadradinho_pintado2 ); // quadradinho menor
            ssd1306_draw_string(&ssd, texto, 71, 20); // o "texto" sera igual ao valor conversao_porcentagem_y porem em caractere
            ssd1306_draw_string(&ssd, "F", 97, 20); // desenha o caracter de "%"
            ssd1306_send_data(&ssd); // Atualiza o display
            alerta_nivel_alto();

        }
        if (conversao_porcentagem_y==100){ //if, haver com a posicao do caractere da porcentagem "%"
            if (ativar_matriz){
                draw_composite_led_matrix(conversao_porcentagem_y);
            }
            ssd1306_fill_region(&ssd, 1, 1, 38, 61, conversao_nivel_y);
            ssd1306_draw_string(&ssd, "NIVEL: ", 64, 2);
            ssd1306_draw_string(&ssd, "B1", 85, 39); 
            ssd1306_draw_string(&ssd, "B2", 85, 53); 
            ssd1306_rect(&ssd, 0, 0, 40, 63,cor,!cor);
            ssd1306_rect(&ssd, 103, 70, 8, 8, 1,quadradinho_pintado1); // quadradinho menor
            ssd1306_rect(&ssd, 117, 70, 8, 8, 1,quadradinho_pintado2 ); // quadradinho menor
            ssd1306_draw_string(&ssd, texto, 71, 20); // o "texto" sera igual ao valor convertido porem em caractere
            ssd1306_draw_string(&ssd, "F", 97, 20); // desenha o caracter de "%", caracter 'F' foi alterado para desenhar o "%"
            ssd1306_send_data(&ssd); // Atualiza o display
            pwm_set_gpio_level(LEDg,0); // apaga o led quando a caixa estiver com volume de agua ok
            pwm_set_gpio_level(LEDr,0); // apaga o led quando a caixa estiver com volume de agua ok

        }
        if (conversao_porcentagem_y<100 && conversao_porcentagem_y>=10){ // if, haver com a posicao do caractere da porcentagem "%"
            if (ativar_matriz){
                draw_composite_led_matrix(conversao_porcentagem_y);
            }
            ssd1306_fill_region(&ssd, 1, 1, 38, 61, conversao_nivel_y);
            ssd1306_draw_string(&ssd, "NIVEL: ", 64, 2);
            ssd1306_draw_string(&ssd, "B1", 85, 39); 
            ssd1306_draw_string(&ssd, "B2", 85, 53); 
            ssd1306_rect(&ssd, 0, 0, 40, 63,cor,!cor);
            ssd1306_rect(&ssd, 103, 70, 8, 8, 1,quadradinho_pintado1); // quadradinho menor
            ssd1306_rect(&ssd, 117, 70, 8, 8, 1,quadradinho_pintado2 ); // quadradinho menor
            ssd1306_draw_string(&ssd, texto, 73, 20); // valor de nivel em porcentagem
            ssd1306_draw_string(&ssd, "F", 92, 20); // alterado posicao da "%"
            ssd1306_send_data(&ssd); 
            pwm_set_gpio_level(LEDg,0); // apaga o led quando a caixa estiver com volume de agua ok
            pwm_set_gpio_level(LEDr,0); // apaga o led quando a caixa estiver com volume de agua ok
  
        }
        if (conversao_porcentagem_y<10){ // if, haver com a posicao do caractere da porcentagem "%"
            ssd1306_fill_region(&ssd, 1, 1, 38, 61, conversao_nivel_y);
            ssd1306_draw_string(&ssd, "NIVEL: ", 64, 2);
            ssd1306_draw_string(&ssd, "B1", 85, 39); 
            ssd1306_draw_string(&ssd, "B2", 85, 53); 
            ssd1306_rect(&ssd, 0, 0, 40, 63,cor,!cor);
            ssd1306_rect(&ssd, 103, 70, 8, 8, 1,quadradinho_pintado1); // quadradinho menor
            ssd1306_rect(&ssd, 117, 70, 8, 8, 1,quadradinho_pintado2 ); // quadradinho menor
            ssd1306_draw_string(&ssd, texto, 73, 20);
            ssd1306_draw_string(&ssd, "F", 86, 20); // alterado posicao
            ssd1306_send_data(&ssd);
            alerta_nivel_baixo();

        }

        //printf("Nivel ADC_y: %d --> valor em porcentagem: %d%%\n", VRY_value, conversao_porcentagem_y); ---> DEBUG

    }
}


void play_note(int frequency, int duration, int pin) // funcao ativacao dos buzzers
{
    if (frequency == 0) {
        sleep_ms(duration);  // Pausa (silêncio)
        return;
    }

    int delay = 1000000 / frequency / 2; // Meio ciclo da frequência
    int cycles = (frequency * duration) / 1000;

    for (int i = 0; i < cycles; i++) {
        gpio_put(pin, 1);
        sleep_us(delay);
        gpio_put(pin, 0);
        sleep_us(delay);
    }
}


void alerta_nivel_baixo() 
{  // ativa o buzzer quando a agua estiver baixa
    if(ativacao_buzzer){
        play_note(440, 50, buzzer1); // Oscilação do som
    } else{
        pwm_set_gpio_level(LEDr, 200); // indica que acionou o motor para encher a caixa (led vermelho)
    }
    
}

void alerta_nivel_alto() 
{  // ativa o buzzer quando a agua estiver alta
    if(ativacao_buzzer){
    play_note(200, 50, buzzer2);  
    play_note(300, 50, buzzer2);  // Oscilação do som
    } else {
    pwm_set_gpio_level(LEDg,200); // indica que acionou o motor para tirar agua da caixa (led azul)
    }
}

void gpio_irq_handler(uint gpio, uint32_t events)
{

    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time >= 350000){ // 350ms

        last_time = current_time; // atualiza o tempo do last_time

        if(gpio == BotaoA){
            ativar_matriz = !ativar_matriz;
            if(ativar_matriz==0){
                clear_matrix();
                quadradinho_pintado1 = 0;
            } else{
                draw_composite_led_matrix(conversao_porcentagem_y);
                quadradinho_pintado1 = 1;
            }
        }
        if(gpio == BotaoB){
            ativacao_buzzer = !ativacao_buzzer; // desativa/ativa os buzzers
            if(ativacao_buzzer == 0){
                quadradinho_pintado2 = 1;
            } else{
                quadradinho_pintado2 = 0;
            }
        }
        if(gpio == BotaoJY){
    
        }

    }

}

void set_matrix_leds(uint32_t colors[NUM_PIXELS]) { // funcao referente a atualizacao da matriz de leds 5x5
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(colors[i]);
    }
}

void draw_composite_led_matrix(int nivel) { // funcao referente a atualizacao matriz de leds 5x5 RGB
    uint32_t final_colors[NUM_PIXELS];
    
    // Inicializa todos os LEDs como apagados
    for (int i = 0; i < NUM_PIXELS; i++) {
        final_colors[i] = 0;
    }
    
    // Sempre desenha a borda (buffer0) com a cor branca (ou de sua escolha)
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (led_buffer0[i]) {
            final_colors[i] = urgb_u32(20, 20, 20);
        }
    }
    
    // Agora, de acordo com o nível, sobrepomos as cores dos estágios:
    if (nivel >= 100) { 
        // Estágio completo: 100%
        for (int i = 0; i < NUM_PIXELS; i++) {
            if (led_buffer1[i]) final_colors[i] = urgb_u32(20, 0, 0);   // 25% – vermelho
            if (led_buffer2[i]) final_colors[i] = urgb_u32(20, 0, 20);  // 50% – violeta
            if (led_buffer3[i]) final_colors[i] = urgb_u32(0, 0, 21);   // 75% – azul escuro
            if (led_buffer4[i]) final_colors[i] = urgb_u32(0, 20, 0); // 100% – verde
        }
    } else if (nivel > 75) {
        // Nível entre 75 e 100%: mostre até o estágio 75%
        for (int i = 0; i < NUM_PIXELS; i++) {
            if (led_buffer1[i]) final_colors[i] = urgb_u32(20, 0, 0);
            if (led_buffer2[i]) final_colors[i] = urgb_u32(20, 0, 20);
            if (led_buffer3[i]) final_colors[i] = urgb_u32(0, 0, 21);
        }
    } else if (nivel >= 50) {
        // Nível entre 50 e 75%: mostre até o estágio 50%
        for (int i = 0; i < NUM_PIXELS; i++) {
            if (led_buffer1[i]) final_colors[i] = urgb_u32(20, 0, 0);
            if (led_buffer2[i]) final_colors[i] = urgb_u32(20, 0, 20);
        }
    } else if (nivel > 25) {
        // Nível entre 25 e 50%: mostre apenas o primeiro estágio (25%)
        for (int i = 0; i < NUM_PIXELS; i++) {
            if (led_buffer1[i]) final_colors[i] = urgb_u32(20, 0, 0);
        }
    }
    // Se nivel < 25, mantém apenas a borda (ou nenhum LED aceso, conforme sua lógica)
    
    // Atualiza a matriz com o buffer composto:
    set_matrix_leds(final_colors);
}

void clear_matrix() { // funcao que desativa a matriz de leds 5x5
    uint32_t colors[NUM_PIXELS];
    for (int i = 0; i < NUM_PIXELS; i++) {
        colors[i] = 0; // Desliga cada LED
    }
    set_matrix_leds(colors);
}