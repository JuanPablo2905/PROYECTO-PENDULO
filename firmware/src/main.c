#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <math.h>
#include <stdlib.h>

#define STEP_PIN 19
#define DIR_PIN 18
#define BUTTON_PIN 3
#define POT_PIN 26
#define KP 25
#define KI 0.3
#define KD 0.1
#define THRESHOLD 0
#define ADC_UPPER_LIMIT 3500
#define ADC_LOWER_LIMIT 972

volatile float equilibrium_position = 0.0;
volatile bool is_equilibrium_set = false;
volatile bool system_paused = false;

float integral_error = 0.0;
float previous_error = 0.0;

float read_pendulum_position() {
    return adc_read() * 360.0 / 4096.0;
}

void setup() {
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);
    
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);
    
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_down(BUTTON_PIN);  // Configura pull-down para el botón

    adc_init();                 // Inicializa el ADC
    adc_gpio_init(POT_PIN);     // Inicializa el pin del potenciómetro
    adc_select_input(0);        // Selecciona el canal de lectura del ADC
}

void button_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        equilibrium_position = adc_read() * 360.0 / 4096.0;
        is_equilibrium_set = true;
        system_paused = false;
        integral_error = 0.0;
        previous_error = 0.0;
        printf("\rPosición de equilibrio definida: %.2f grados\n", equilibrium_position);
        fflush(stdout);
    }
}

void control_motor(float error) {
    integral_error += error * 0.001;
    float derivative_error = (error - previous_error) / 0.001;
    float control_signal = (KP * error) + (KI * integral_error) + (KD * derivative_error);

    if (fabs(control_signal) > 1500) {
        control_signal = 1500;
    }

    if (fabs(error) > THRESHOLD) {
        gpio_put(DIR_PIN, control_signal > 0 ? 1 : 0);

        for (int i = 0; i < 4; i++) {
            gpio_put(STEP_PIN, 1);
            sleep_us(1);
            gpio_put(STEP_PIN, 0);
            sleep_us(1500 - control_signal);
        }
    }

    previous_error = error;
}

int main() {
    stdio_init_all();
    setup();
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);

    while (true) {
        if (system_paused) {
            printf("Sistema pausado por caída. Presiona el botón para reiniciar.\n");
            fflush(stdout);
            sleep_ms(100);
            continue;
        }

        float current_position = read_pendulum_position();

        uint16_t adc_value = adc_read();
        if (adc_value > ADC_UPPER_LIMIT || adc_value < ADC_LOWER_LIMIT) {
            system_paused = true;
            is_equilibrium_set = false;
            printf("\r¡El péndulo se ha caído! Sistema detenido. Presiona el botón para reiniciar.\n");
            fflush(stdout);
            continue;
        }

        if (is_equilibrium_set) {
            float error = equilibrium_position - current_position;
            control_motor(error);
        } else {
            printf("\rEsperando configuración de posición de equilibrio... Lectura ADC: %.2f\n", current_position);
        }

        // Imprimir los valores de las dos funciones que queremos graficar:
        printf("PendulumAngle:%.2f,ReferenceAngle:%.2f\n", current_position, equilibrium_position);

        fflush(stdout);
        sleep_ms(1);
    }

    return 0;
}