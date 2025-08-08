/*
    Cria uma conexão HTTP, que envia uma string no tópico /pico_publisher, no nó pico_node

*/
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include "lib/pico_uart/pico_uart_transports.h"

#include "lib/oled/ssd1306.h"
#include "hardware/i2c.h"

#define I2C_SDA 14
#define I2C_SCL 15

#define LED_PIN_G 11          // Define o pino do LED
#define LED_PIN_R 13
#define WIFI_SSID "***"  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASS "******" // Substitua pela senha da sua rede Wi-Fi

// Buffer para respostas HTTP
#define HTTP_RESPONSE "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" \
                      "<!DOCTYPE html><html><body>" \
                      "<h1>Controle do Tello</h1>" \
                      "<p><a href=\"/takeoff\">Takeoff</a></p>" \
                      "<p><a href=\"/land\">Land</a></p>" \
                      "</body></html>\r\n"


rcl_publisher_t publisher;
std_msgs__msg__String msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    msg.data.size = strlen(msg.data.data);
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
}

// Função de callback para processar requisições HTTP
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Cliente fechou a conexão
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Processa a requisição HTTP
    char *request = (char *)p->payload;

    if (strstr(request, "GET /takeoff")) {
        gpio_put(LED_PIN_G, 1);  // takeoff ---> led verde
        gpio_put(LED_PIN_R, 0);
        msg.data.data = "takeoff";

    } else if (strstr(request, "GET /land")) {
        gpio_put(LED_PIN_G, 0);  // land ---> led vermelho
        gpio_put(LED_PIN_R, 1);
        msg.data.data = "land";
    }
    else if (strstr(request, "GET /")) {
        msg.data.data = "none";
        gpio_put(LED_PIN_G, 0);  // nada ---> led desligado
        gpio_put(LED_PIN_R, 0);
    }

    // Envia a resposta HTTP
    tcp_write(tpcb, HTTP_RESPONSE, strlen(HTTP_RESPONSE), TCP_WRITE_FLAG_COPY);

    // Libera o buffer recebido
    pbuf_free(p);

    return ERR_OK;
}

// Callback de conexão: associa o http_callback à conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);  // Associa o callback HTTP
    return ERR_OK;
}

// Função de setup do servidor TCP
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }

    // Liga o servidor na porta 80
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);  // Coloca o PCB em modo de escuta
    tcp_accept(pcb, connection_callback);  // Associa o callback de conexão

    printf("Servidor HTTP rodando na porta 80...\n");
}

// Inicializa o display OLED
void init_display(uint8_t *ssd, struct render_area *frame_area) {
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();

    // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
    *frame_area = (struct render_area) {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(frame_area);

    // zera o display inteiro
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, frame_area);
}

// Desenha o texto no display
void draw_text(uint8_t *ssd, struct render_area frame_area, char *text[], int count_of) {
    memset(ssd, 0, ssd1306_buffer_length);
    int y = 0;
    for (uint i = 0; i < count_of; i++)
    {
        ssd1306_draw_string(ssd, 0, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}


int main() {
    stdio_init_all();  // Inicializa a saída padrão
    // Inicializa o display OLED
    uint8_t ssd[ssd1306_buffer_length];
    struct render_area frame_area;
    init_display(ssd, &frame_area);  
    char *text[] = {"Iniciando", "o micro-ROS"};    
    draw_text(ssd, frame_area, text, 2);
    
    printf("Iniciando o micro-ROS\n");
    // inicia o microros
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    while (true) {
        rcl_ret_t ret = rmw_uros_ping_agent(1000, 1);
        if (ret == RCL_RET_OK) break;

        text[0] = "Procurando";   
        text[1] = "agente";
        draw_text(ssd, frame_area, text, 2);

        printf("Agente não detectado. Tentando novamente...\n");
        sleep_ms(1000);
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "pico_publisher");

    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    printf("Micro-ros inicializado com sucesso\n");

    printf("Iniciando servidor HTTP\n");
    // Inicializa o Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    text[0] = "Conectando ao";   
    text[1] = "Wi-Fi";
    draw_text(ssd, frame_area, text, 2);
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        text[0] = "Falha ao conectar";   
        draw_text(ssd, frame_area, text, 1);

        return 1;
    }else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);

        static char temp[16];
        snprintf(temp, sizeof(temp), "%d.%d.%d.%d", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

        text[0] = "Endereco IP";
        text[1] = temp;
        draw_text(ssd, frame_area, text, 2);

        printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    printf("Wi-Fi conectado!\n");
    // printf("Para ligar ou desligar o LED acesse o Endereço IP seguido de /led/on ou /led/off\n");

    // Configura o LED como saída
    gpio_init(LED_PIN_G);
    gpio_init(LED_PIN_R);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);

    // Inicia o servidor HTTP
    start_http_server();

    
    // Loop principal
    while (true) {
        cyw43_arch_poll();  // Necessário para manter o Wi-Fi ativo
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // Atualiza o display
        text[2] = msg.data.data;
        draw_text(ssd, frame_area, text, 3);

        sleep_ms(100);
    }

    cyw43_arch_deinit();  // Desliga o Wi-Fi (não será chamado, pois o loop é infinito)
    return 0;
}
