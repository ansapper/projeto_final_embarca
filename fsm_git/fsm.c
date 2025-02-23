// 
// André Neves Sapper - Projeto final Embarcatech - Turma de Campinas - 2025
// Projeto: radar utilizando BitDog Lab
// 


#include <stdio.h>             // printf() para enviar dados ao console
#include <stdint.h>            // uint16_t
#include <stdlib.h>            // abs()
#include <stdbool.h>           // bool
#include <string.h>            // i2c
#include <ctype.h>             // i2c
#include <math.h>              // sqrt
#include "pico/time.h"         // tempo
#include "pico/stdlib.h"       // Contém funções básicas do RP2040 como controle de GPIO e temporização
#include "pico/binary_info.h"  // i2c
#include "hardware/adc.h"      // adc
#include "inc/ssd1306.h"       // i2c


// Definição dos pinos
#define BTN_A       5  //Botão A (Manual)
#define BTN_B       6  //Botão B (Automatico)
#define LED_MANU   11  //LED verde
#define LED_AUTO   12  //LED azul
#define ADC2       28  //Microfone
#define I2C_SDA    14  //I2C OLED
#define I2C_SCL    15  //I2C OLED


// Estados da FSM
typedef enum {
    FSM_RESET,
    FSM_LEARN,
    FSM_IDLE,
    FSM_MANUAL,
    FSM_AUTO,
} FSM_STATE;

// Estado inicial/atual do sistema (FSM)
// "A variável global fsm_state, do tipo FSM_STATE, começa no estado FSM_RESET."
FSM_STATE fsm_state = FSM_RESET;

// para DEBUG
const char *fsm_state_names[] = {
    "FSM_RESET",
    "FSM_LEARN",
    "FSM_IDLE",
    "FSM_MANUAL",
    "FSM_AUTO"
};


// Estados do ALGO (Algoritmo do radar)
typedef enum {
    ALGO_INIT,
    ALGO_WAIT_TRIGGER,
    ALGO_SAMPLING,
    ALGO_CALCULATE
} ALGO_STATE;

ALGO_STATE algo_state = ALGO_INIT; // Estado inicial/atual

// para DEBUG
const char *algo_state_names[] = {
    "ALGO_INIT",
    "ALGO_WAIT_TRIGGER",
    "ALGO_SAMPLING",
    "ALGO_CALCULATE"
};





// Variáveis globais
volatile uint16_t sample = 0;       // Amostra do ADC-microfone
volatile uint16_t silence = 0;      // Valor de silêncio
volatile uint16_t trigger = 0;      // Trigger = 3 * silence
volatile uint16_t peak = 0;         // Valor máximo da amostra
volatile uint16_t peak_half = 0;    // Valor da amostra quando cai abaixo de metade do pico
volatile float t1 = 0;              // Tempo do peak
volatile float t2 = 0;              // Tempo do peak/2
volatile float speed_kmh = 0;       // velocidade do radar
volatile uint16_t flag = 0;         // Flag para controlar transição
static char oled_l1[16];            // OLED Linha 1
static char oled_l2[16];            // OLED Linha 2
static char oled_l3[16];            // OLED Linha 3
static char oled_l4[16];            // OLED Linha 4
uint16_t modo = 0;                  // 0=off  1=manual  2=automatico
volatile uint16_t offset = 0;       // Tentativa de corrigir o offset do ADC


// variaveis globais auxiliares
uint32_t sum = 0;
uint32_t count = 0;

static volatile uint botaoA_flag = false;  // Flag IRQ Botao A
static volatile uint botaoB_flag = false;  // Flag IRQ Botao B


// para DEBUG no terminal
static int _modo = -1;
static int _silence = -1;
static int _trigger = -1;
static int _sample = -1;
static int _peak = -1;
static int _peak_half = -1;
static float _t1 = -1.0;
static float _t2 = -1.0;
static float _speed_kmh = -1.0;



// Protótipos das funções
void init_hardware(void);
uint16_t learning(void);
static void gpio_callback(uint gpio, uint32_t events);  // IRQ GPIO
void display_message(const char *line1, const char *line2, const char *line3, const char *line4);
float calculate_speed(float delta);
uint16_t read_sample(void);
void get_time_t1(void);
void get_time_t2(void);

void FSM_State(FSM_STATE fsm_state);
FSM_STATE FSM_NextState(FSM_STATE fsm_state);

void ALGO_State(ALGO_STATE algo_state);
ALGO_STATE ALGO_NextState(ALGO_STATE algo_state);






//##################################################################################################################
//                                                                                            Função principal 
//##################################################################################################################
void main() {
    stdio_init_all(); //Inicialização geral
    init_hardware();  //Inicialização do hardware

    // debug
    sleep_ms(5000);

    //Loop infinito
    while (true) {

        // Atualiza o estado da FSM
        fsm_state = FSM_NextState(fsm_state);
        
        // Executa ações do estado atual da FSM
        FSM_State(fsm_state);


        // DEBUG no terminal (so imprime se mudou)
        if (modo != _modo || silence != _silence || trigger != _trigger || sample != _sample || peak != _peak || peak_half != _peak_half ||
        t1 != _t1 || t2 != _t2 || speed_kmh != _speed_kmh) {
            printf("%-11s %-19s modo=%-1d  silence=%-3d  trigger=%-3d  sample=%-4d  peak=%-4d peak_half=%-4d  t1=%-4.2f   t2=%-4.2f   kmh=%-3.2f\n", 
            fsm_state_names[fsm_state], algo_state_names[algo_state], modo, silence, trigger, sample, peak, peak_half, t1, t2, speed_kmh);

            // atualiza valores anteriores
            _modo = modo;
            _silence = silence;
            _trigger = trigger;
            _sample = sample;
            _peak = peak;
            _peak_half = peak_half;
            _t1 = t1;
            _t2 = t2;
            _speed_kmh = speed_kmh;
        }
    }
}


//##################################################################################################################
//                                                                                          FSM Next State logic 
//##################################################################################################################
FSM_STATE FSM_NextState(FSM_STATE fsm_state) {
    switch (fsm_state) {
        case FSM_RESET:
            if (botaoA_flag) {
                sleep_ms(500);         // delay para compensar o click do botão
                botaoA_flag = false;   // reseta a flag
                return FSM_LEARN; }    // vai para o estado..

            if (botaoB_flag) {
                sleep_ms(500);         // delay para compensar o click do botão
                botaoB_flag = false;   // reseta a flag
                return FSM_LEARN; }    // vai para o estado..

            return FSM_RESET;          // Mantém o estado atual


        case FSM_LEARN:
            // Apos o treinamento, passa para o estado FSM_IDLE automaticamente
            return FSM_IDLE;


        case FSM_IDLE:
            if (botaoA_flag) {
                sleep_ms(500);          // delay para compensar o click do botão
                botaoA_flag = false;    // reseta a flag
                modo = 1;               // ativa modo MANUAL
                return FSM_MANUAL; }    // vai para o estado..

            if (botaoB_flag) {
                sleep_ms(500);          // delay para compensar o click do botão
                botaoB_flag = false;    // reseta a flag
                modo = 2;               // ativa modo AUTOMATICO
                return FSM_AUTO;   }    // vai para o estado..

            return FSM_IDLE;            // Mantém o estado atual


        case FSM_MANUAL:
            if (botaoA_flag) {
                botaoA_flag = false;    // reseta a flag
                modo = 0;               // desativa modo MANUAL
                return FSM_IDLE; }      // vai para o estado..

            if (botaoB_flag) {
                botaoB_flag = false;    // reseta a flag
                modo = 0;               // desativa modo MANUAL
                return FSM_IDLE; }      // vai para o estado..

            if (modo == 0) {
                return FSM_IDLE;
            }
        
            return FSM_MANUAL;          // Mantém o estado atual

            
        case FSM_AUTO:
            if (botaoA_flag) {
                botaoA_flag = false;    // reseta a flag
                return FSM_IDLE; }      // vai para o estado..

            if (botaoB_flag) {
                botaoB_flag = false;    // reseta a flag
                return FSM_IDLE; }      // vai para o estado..

            return FSM_AUTO;            // mantem o estado atual
        

        default:
            return FSM_RESET;           // se houver algum erro inesperado, reinicia
    }
}




//##################################################################################################################
//                                                                                                FSM State Logic
//##################################################################################################################
void FSM_State(FSM_STATE fsm_state) {
    switch (fsm_state) {
        case FSM_RESET:
            display_message(" Hardware OK!", NULL, " Aperte A ou B", NULL);
            break;


        case FSM_LEARN:
            display_message("Calibrando..", NULL, "S i l e n c i o", NULL);
            
            // Chama a função para amostrar o silencio (silence)
            learning();
            break;


        case FSM_IDLE:
            gpio_put(LED_MANU, 0); //Desliga o LED
            gpio_put(LED_AUTO, 0); //Desliga o LED

            sprintf(oled_l3, "Silence   %d", silence);
            sprintf(oled_l4, "Trigger   %d", trigger);
            display_message("Mic Calibrado.", "", oled_l3, oled_l4);
            break;


        case FSM_MANUAL:
            gpio_put(LED_MANU, 1); //Liga o LED verde

            sprintf(oled_l1, "Si %3d  Tr %3d", silence, trigger);
            sprintf(oled_l2, "Pe %3d  Ph %3d", peak, peak_half);
            sprintf(oled_l3, "Sa %3d", sample);
            sprintf(oled_l4, "Km %2f", speed_kmh);
            //sprintf(oled_l4, "co %6d", count);  // debug
            display_message(oled_l1, oled_l2, oled_l3, oled_l4);
             
            // Atualiza o estado do ALGO 
            algo_state = ALGO_NextState(algo_state);
        
            // Executa ações do estado atual do ALGO
            ALGO_State(algo_state);
            break;


        case FSM_AUTO:
            gpio_put(LED_AUTO, 1); //Liga o LED azul
            
            // Medição automática (loop)
            sprintf(oled_l1, "Si %3d  Tr %3d", silence, trigger);
            sprintf(oled_l2, "Pe %3d  Ph %3d", peak, peak_half);
            sprintf(oled_l3, "Sa %3d", sample);
            sprintf(oled_l4, "Km %2f", speed_kmh);
            //sprintf(oled_l4, "co %6d", count);  // debug
            display_message(oled_l1, oled_l2, oled_l3, oled_l4);
             
            // Atualiza o estado do ALGO 
            algo_state = ALGO_NextState(algo_state);
        
            // Executa ações do estado atual do ALGO
            ALGO_State(algo_state);
            break;
    }
}



//##################################################################################################################
//                                                                                         ALGO next state logic 
//##################################################################################################################
ALGO_STATE ALGO_NextState(ALGO_STATE algo_state) {
    switch (algo_state) {
        case ALGO_INIT:
            if (modo == 1 || modo == 2) {
                return ALGO_WAIT_TRIGGER;
            }
            return ALGO_INIT;
            break;

        case ALGO_WAIT_TRIGGER:
            if (sample > trigger) {
                return ALGO_SAMPLING;
            }
            return ALGO_WAIT_TRIGGER;
            break;

        case ALGO_SAMPLING:
            if (sample > peak) {
                return ALGO_SAMPLING;
            } else {
                if (flag == 0) {
                    if (sample < peak/2) {
                        return ALGO_SAMPLING;
                    }
                } else {
                    if (sample < silence) {
                        return ALGO_CALCULATE;
                    }
                }
            }
            return ALGO_SAMPLING;
            break;

        case ALGO_CALCULATE:
            if (modo == 1) {           // se estiver no modo MANUAL, desativa e retorna pro estado inicial
                sleep_ms(3000);        // exibe a tela com as respostas
                modo = 0;              //desativa modo MANUAL
                return ALGO_INIT;
            } else {
                if (modo == 2) {       // se estiver no modo AUTO, mantem modo=2 e continua medindo
                    sleep_ms(3000);    // exibe a tela com as respostas
                    return ALGO_INIT;  // Após calcular, volta para ALGO_INIT
                }
            }

        default:
            return ALGO_INIT; // Estado padrão
    }
}

//##################################################################################################################
////                                                                                             ALGO state logic
//##################################################################################################################
void ALGO_State(ALGO_STATE algo_state) {
    switch (algo_state) {
        case ALGO_INIT:
            sample = 0;
            peak = 0;
            peak_half = 0;
            flag = 0;
            t1 = 0;
            t2 = 0;
            speed_kmh = 0;
            break;


        case ALGO_WAIT_TRIGGER:
            // Aguarda o trigger
            read_sample();
            
            // Se trigger
            if (sample > trigger) {
                peak = sample;        // peak
                get_time_t1();        // t1
            }
            break;


        case ALGO_SAMPLING:
            // procura o peak
            read_sample();

            if (sample > peak) {
                peak = sample;       // Atualiza o pico
                get_time_t1();       // Registra o tempo do pico
                flag = 0;            // Reseta a flag
            } else {
                if (flag == 0) {
                    if (sample < peak/2) {
                        flag = 1;             // Ativa a flag
                        peak_half = sample;   // Registra a metade do pico
                        get_time_t2();        // Registra o tempo da metade do pico
                    }
                }
            }
            break;


        case ALGO_CALCULATE:
            {
                float delta = t2 - t1;              // Calcula o delta de tempo
                speed_kmh = calculate_speed(delta); // Calcula a velocidade
            }
            break;
    }
}





//##################################################################################################################
//                                                                                    Inicialização do Hardware
//##################################################################################################################
void init_hardware(void) {

    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

    // Inicializa e configura o ADC (Microfone)
    adc_init();
    adc_gpio_init(ADC2);   // Inicializa o Microfone
    adc_select_input(2);   // Seleciona o canal 2 do ADC
    adc_set_clkdiv(12.5);  // define taxa de amostragem para 40 kSps (é 2*fmax do sinal de audio..nyquist)
    
    //Inicialização do pinos (GPIOs)
    gpio_init(BTN_A);     // inicializa o Botão A
    gpio_init(BTN_B);     // inicializa o Botão B
    gpio_init(LED_MANU);  // inicializa o LED azul
    gpio_init(LED_AUTO);  // inicializa o LED verde

    //Configuração dos pinos como entrada e saída
    gpio_set_dir(BTN_A, GPIO_IN);     // Botão é entrada
    gpio_set_dir(BTN_B, GPIO_IN);     // Botão é entrada
    gpio_set_dir(LED_MANU, GPIO_OUT); // LED é saída
    gpio_set_dir(LED_AUTO, GPIO_OUT); // LED é saída

    gpio_pull_up(BTN_A);  // pull-up interno
    gpio_pull_up(BTN_B);  // pull-up interno

    //IRQ Registro global do callback (chama somente 1x)
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback); 
    gpio_set_irq_enabled(BTN_B, GPIO_IRQ_EDGE_FALL, true);   //IRQ para Botão B

    gpio_put(LED_MANU, 0);  // garante que o LED esteja desligado, a princípio
    gpio_put(LED_AUTO, 0);  // garante que o LED esteja desligado, a princípio
}





//##################################################################################################################
//                                                                                            learning silence
//##################################################################################################################
uint16_t learning(void) {
    printf("Estado: FSM_STATE_LEARN - Aprendendo...\n");
    
    // reseta o valor maximo
    // silence = 0;

    // acumulador e contador de 32 bits
    sum = 0;
    count = 0;

    // Amostra o sinal durante 1 segundo
    uint32_t start_time = to_ms_since_boot(get_absolute_time());          // tempo inicial
    while (to_ms_since_boot(get_absolute_time()) - start_time < 1000) {   // processa por 1 segundo
        uint16_t raw_adc_value = adc_read();                              // le o valor do ADC (0 a 4095)
        int16_t value = (int16_t)raw_adc_value - 2048;                    // -2048 a 2047
        uint16_t abs_value = (value < 0) ? -value : value;                // igual abs() - 0 a 2047

        // MEDIA
        //sum = sum + abs_value;    // acumulador
        //count++;                  // contador de amostras

        // RMS
        uint32_t squared_value = value * value;  // quadrado do valor
        sum += squared_value;                    // acumulador
        count++;                                 // 
    }
    // MEDIA
    //silence = (uint16_t)(sum / count);  // calcula a media antes de retornar
    //printf("sum:  %d      count:  %d\n", sum, count);

    // Calcula o valor RMS
    float rms = sqrtf((float)sum / count) ;   // RMS = sqrt(soma dos quadrados / número de amostras)
    trigger = 3 * rms;                        // trigger = 3*silence
    silence = (uint16_t)rms * 1.2f;           // converte RMS para uint16_t e armazena em silence (+ 20% do ventilador :) )

}



//##################################################################################################################
//                                                                                      Amostra do sensor (ADC) 
//##################################################################################################################
uint16_t read_sample(void) {
    //sample=0;
    
    // MEDIA - acumulador e contador de 32 bits
    sum = 0;
    count = 0;

    // Amostra o sinal durante X milisegundos
    uint32_t start_time = to_ms_since_boot(get_absolute_time());        // tempo inicial
    while (to_ms_since_boot(get_absolute_time()) - start_time < 35) {   // tempo de processamento
        uint16_t raw_adc_value = adc_read();                            // le ADC (0 a 4095)
        int16_t value = (int16_t)raw_adc_value - 2048;                  // Faixa de -2048 a 2047
        uint16_t abs_value = (value < 0) ? -value : value;              // igual abs() 

        // MEDIA
        //sum = sum + abs_value;    // acumulador
        //count++;                  // contador de amostras

        // RMS
        uint32_t squared_value = value * value;  // quadrado do valor
        sum += squared_value;                    // acumulador
        count++;                                 // 
        }
    // MEDIA
    //sample = (uint16_t)(sum / count);  // calcula a media antes de retornar
    //printf("sum:  %d      count:  %d\n", sum, count);

    // RMS
    float rms = sqrtf((float)sum / count);   // RMS = sqrt(soma dos quadrados / número de amostras)
    sample = (uint16_t)rms;                  // Converte o RMS para uint16_t e armazena em sample

    //return sample;
}




//##################################################################################################################
//                                                                                         Dsiplay oled message  
//##################################################################################################################
void display_message(const char *line1, const char *line2, const char *line3, const char *line4)
{
    // Se o parâmetro não for NULL, atualiza o buffer correspondente
    if (line1 != NULL) {
        strncpy(oled_l1, line1, sizeof(oled_l1) - 1);
        oled_l1[sizeof(oled_l1) - 1] = '\0';
    }
    if (line2 != NULL) {
        strncpy(oled_l2, line2, sizeof(oled_l2) - 1);
        oled_l2[sizeof(oled_l2) - 1] = '\0';
    }
    if (line3 != NULL) {
        strncpy(oled_l3, line3, sizeof(oled_l3) - 1);
        oled_l3[sizeof(oled_l3) - 1] = '\0';
    }
    if (line4 != NULL) {
        strncpy(oled_l4, line4, sizeof(oled_l4) - 1);
        oled_l4[sizeof(oled_l4) - 1] = '\0';
    }

    // Configura a área de renderização do display
    struct render_area frame_area = {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);

    // Cria e zera o buffer para o display
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);

    // Desenha as duas linhas (as linhas atuais)
    ssd1306_draw_string(ssd, 5, 0,  oled_l1);
    ssd1306_draw_string(ssd, 5, 8,  oled_l2);
    ssd1306_draw_string(ssd, 5, 16, oled_l3);
    ssd1306_draw_string(ssd, 5, 24, oled_l4);

    // Atualiza o display
    render_on_display(ssd, &frame_area);
}






//##################################################################################################################
//                                                                                             METODO ANALITICO
//##################################################################################################################
float calculate_speed(float delta) {
    // distancia entre o sensor e o veiculo (perpendicular)
    float d1 = 5.0;

    // calculo da velocidade em m/s usando a fórmula: v = (d1 * sqrt(3)) / delta
    float speed_mps = (d1 * sqrt(3)) / delta;

    // Converte a velocidade para km/h (1 m/s = 3.6 km/h)
    float speed_kmh = speed_mps * 3.6;

    return speed_kmh; // Retorna a velocidade em km/h
}





//##################################################################################################################
//                                                                                             Callback handler
//##################################################################################################################
static void gpio_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (gpio == BTN_A) {
        // Variável estática para armazenar o tempo do último acionamento do botão A
        static uint32_t last_btn_a_time = 0;
        // debounce de ruido de audio do botao
        if (now - last_btn_a_time < 300)
            return;  // se não passou o tempo minimo, ignora o acionamento (debounce)
        // Atualiza o tempo do último acionamento do botão A
        last_btn_a_time = now;
        
        // Executa ações quando o botão A é considerado "limpo" (sem bouncing)
        printf("Botão A\n");
        botaoA_flag = true;
    }
    else if (gpio == BTN_B) {
        static uint32_t last_btn_b_time = 0;
        if (now - last_btn_b_time < 300)
            return;
        last_btn_b_time = now;
        
        printf("Botão B\n");
        botaoB_flag = true;
    }
}



//##################################################################################################################
//                                                                                                         timer
//##################################################################################################################
// t1 (Timer)
void get_time_t1(void) {
    t1 = to_ms_since_boot(get_absolute_time()) / 1000.0; // converte para segundos
}

// t2 (Timer)
void get_time_t2(void) {
    t2 = to_ms_since_boot(get_absolute_time()) / 1000.0;  // converte para segundos
}
