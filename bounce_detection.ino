#include <Arduino.h>
#include <math.h>

#define SENSOR_PIN 34 // Entrada analógica do sensor MOX
#define BUZZER_PIN 13 // Saída digital para o buzzer (GPIO13)

const float VREF = 3.3; // Tensão de referência do ESP32
const int RESOLUCAO = 4095; // 12 bits

float thalf = 0.2;

const float intervalo_ms = 100; // Leitura a cada 100 ms (10 Hz)
unsigned long ultimo_tempo = 0;

float fs = 1000.0/intervalo_ms;
float alpha = 1 - exp(log(0.5f)/(thalf * fs));
//float alpha = 0.5;
float xs, xs_prev = 0;
float d1s, d1s_prev = 0;

float inicio_bout, fim_bout;
float treshold = 0.0001;
bool detectou_bout = false;
bool bout_verdadeiro = false;
int contador_bouts = 0;


float ewma(float entrada, float anterior, float alpha) {
  return alpha * entrada + (1 - alpha) * anterior;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  delay(1000);
  Serial.println("Monitoramento de gás com buzzer para bouts.");
  Serial.printf("Aplha: %f\n", alpha);
  delay(5000);
}

void loop() {
  unsigned long agora = millis();
  if (agora - ultimo_tempo >= intervalo_ms) {

    int leitura_adc = analogRead(SENSOR_PIN);
    float x = leitura_adc * (VREF / RESOLUCAO); // tensao em V

    xs = ewma(x, xs_prev, alpha); // tensao suavisada // variavel global

    float d1 = (xs - xs_prev) / (intervalo_ms / 1000.0); // dV/dt
    d1s = ewma(d1, d1s_prev, alpha); // variavel global // deixa o sinal da derivada suave denovo para filtrar ruidos

    if (!detectou_bout && (d1s_prev <= 0 && d1s > 0)) // cruzou zero numa curva ascendente
    // segunda derivada positiva
    {
      detectou_bout = true; // marca o inicio do bout
      inicio_bout = d1s_prev;
    }
    else if (detectou_bout && (d1s_prev > 0 && d1s <= 0)) // cruzou zero numa curva descendente?
    // so sera verdadeiro se realmente passar por zero denovo, ou seja, a segunda derivada for negativa
    {
      fim_bout = d1s_prev;
      float amplitude = fim_bout - inicio_bout;

      if (amplitude >= treshold){
        contador_bouts++;
        tone(BUZZER_PIN, 440, 50);
        bout_verdadeiro = true;
      } else bout_verdadeiro = false;

      detectou_bout = false; // acabou a deteccao
    }

    Serial.print("Tensao:");
    Serial.print(x, 3);
    Serial.print("\tSuavizado:");
    Serial.print(xs, 3);
    Serial.print("\tDerivada:");
    Serial.print(d1s, 3);
    Serial.print("\tBout:");
    Serial.println(bout_verdadeiro ? 0.5 : 0.0);

    xs_prev = xs;
    d1s_prev = d1s;
    ultimo_tempo = agora;

    bout_verdadeiro = false;
  }
}