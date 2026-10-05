// =====================================================================
//  Detector de estresse/excitação (GSR + batimentos) - ESP32 + FreeRTOS
//  Easy Pulse v1.1 (saída digital HB OUT) + circuito GSR
//
//  Ligações:
//    Easy Pulse HB OUT  -> GPIO27  (alimente o Easy Pulse com 3,3 V!)
//    Easy Pulse AO      -> só na ponta do osciloscópio (ajuste de ganho)
//    GSR (saída analóg) -> GPIO35
//    Botão de pergunta  -> GPIO0 (botão BOOT da placa, ativo em LOW)
//    LED                -> GPIO2
//
//  Saída serial: linhas iniciadas com '#' são comentários/status;
//  as demais são CSV:
//    tempo_ms,bpm,baseline_bpm,gsr_raw,gsr_fasico_z,gsr_tonico_z,estresse,estado,evento
// =====================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <stdarg.h>
#include <math.h>

// ---------- Pinos ----------
#define HB_PIN          27   // HB OUT do Easy Pulse (pulso por batimento)
#define GSR_PIN         35   // ADC1_CH7
#define LED_PIN         2
#define BTN_PIN         0    // marca o início de cada pergunta

// ---------- PPG / batimentos ----------
#define PPG_MIN_INTERVALO_US  300000UL   // 200 BPM (debounce da ISR)
#define PPG_MAX_INTERVALO_US  2000000UL  // 30 BPM
#define PPG_DESCARTE_INICIAL  5          // batimentos iniciais instáveis
#define PPG_N_CALIB           15         // amostras para o baseline (mediana)
#define PPG_MIN_CALIB         5          // mínimo aceito se der timeout
#define PPG_TIMEOUT_CALIB_MS  40000
#define BPM_AUMENTO_PERC      15.0f      // aumento sobre o baseline

// ---------- GSR ----------
#define GSR_PERIODO_MS        50         // 20 Hz
#define GSR_TEMPO_CALIB_MS    25000
#define GSR_IGNORAR_INICIO_MS 3000       // filtros se estabilizando
#define GSR_INVERTIDO         false      // true se o ADC SOBE quando a condutância CAI
#define GSR_ALPHA_RAPIDO      0.3f       // suavização do ruído (~0,15 s)
#define GSR_ALPHA_LENTO       0.02f      // linha de base lenta (~2,5 s)
#define GSR_K_FASICO          4.0f       // pico fásico: desvios acima do ruído em repouso
#define GSR_LIMIAR_Z_TONICO   3.0f       // nível tônico: desvios acima do baseline
#define GSR_STD_MIN           2.0f       // piso do desvio (contagens ADC)

// ---------- Decisão ----------
#define DADO_VELHO_MS           3000     // descarta dados mais velhos que isso
#define JANELA_COINCIDENCIA_MS  6000     // BPM e GSR altos dentro dessa janela
#define JANELA_PERGUNTA_MS      8000     // análise após apertar o botão
#define CSV_PERIODO_MS          500

// ---------- Estruturas das filas ----------
struct BeatMsg { uint32_t tsMs; float bpm; };                       // ISR -> task_ppg
struct PpgMsg  { uint32_t tsMs; float bpm; float baseline; };       // task_ppg -> análise
struct GsrMsg  { uint32_t tsMs; float raw; float fast;
                 float zFasico; float zTonico; };                   // task_gsr -> análise

QueueHandle_t beatQueue, ppgQueue, gsrQueue;
SemaphoreHandle_t serialMutex;

// ---------- Utilitários ----------
// Serial protegida por mutex: várias tarefas podem imprimir sem embaralhar.
void serialPrintf(const char *fmt, ...) {
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println(buf);
        xSemaphoreGive(serialMutex);
    }
}

// Mediana (arrays pequenos)
float mediana(const float *v, int n) {
    float t[32];
    if (n > 32) n = 32;
    for (int i = 0; i < n; i++) t[i] = v[i];
    for (int i = 1; i < n; i++) {           // insertion sort
        float x = t[i];
        int j = i - 1;
        while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
        t[j + 1] = x;
    }
    return (n % 2) ? t[n / 2] : (t[n / 2 - 1] + t[n / 2]) / 2.0f;
}

// Idade com sinal (evita underflow quando o timestamp é um pouco "futuro")
static inline int32_t idade(uint32_t agora, uint32_t ts) {
    return (int32_t)(agora - ts);
}

// ---------- ISR: batimento detectado pelo HB OUT ----------
static volatile uint32_t ultimoBatimentoUs = 0;

void IRAM_ATTR isr_batimento() {
    uint32_t agoraUs = micros();
    uint32_t intervalo = agoraUs - ultimoBatimentoUs;

    // Debounce: ignora pulsos mais próximos que 300 ms (> 200 BPM)
    if (ultimoBatimentoUs != 0 && intervalo < PPG_MIN_INTERVALO_US) return;

    bool valido = (ultimoBatimentoUs != 0) && (intervalo <= PPG_MAX_INTERVALO_US);
    ultimoBatimentoUs = agoraUs;
    if (!valido) return;  // primeiro batimento ou intervalo longo demais

    BeatMsg m = { millis(), 60000000.0f / (float)intervalo };
    BaseType_t acordou = pdFALSE;
    xQueueSendFromISR(beatQueue, &m, &acordou);
    if (acordou) portYIELD_FROM_ISR();
}

// ---------- Tarefa: BPM + calibração do baseline ----------
void task_ppg(void *pvParameters) {
    float calib[PPG_N_CALIB];
    int nCalib = 0, descartados = 0;
    float baseline = 0.0f;
    bool calibrando = true;

    float hist[3];                 // filtro de mediana de 3 batimentos
    int nHist = 0, idxHist = 0;

    uint32_t inicio = millis();
    uint32_t ultimoBatimento = 0;
    uint32_t ultimoAviso = inicio;

    serialPrintf("# [PPG] Calibrando baseline de BPM. Fique parado e relaxe. Coloque o dedo no sensor.");

    for (;;) {
        BeatMsg b;
        if (xQueueReceive(beatQueue, &b, pdMS_TO_TICKS(200)) == pdTRUE) {
            ultimoBatimento = b.tsMs;

            if (calibrando) {
                if (descartados < PPG_DESCARTE_INICIAL) {
                    descartados++;                       // descarta os primeiros (instáveis)
                } else if (nCalib < PPG_N_CALIB) {
                    calib[nCalib++] = b.bpm;
                    serialPrintf("# [PPG] Calibracao %d/%d (BPM=%.0f)", nCalib, PPG_N_CALIB, b.bpm);
                }
            } else {
                hist[idxHist] = b.bpm;
                idxHist = (idxHist + 1) % 3;
                if (nHist < 3) nHist++;
                float bpmFiltrado = (nHist < 3) ? b.bpm : mediana(hist, 3);

                PpgMsg m = { b.tsMs, bpmFiltrado, baseline };
                xQueueSend(ppgQueue, &m, 0);
            }
        }

        uint32_t agora = millis();

        // Aviso de falta de sinal (calibrando ou não)
        uint32_t ref = ultimoBatimento ? ultimoBatimento : inicio;
        if (idade(agora, ref) > 5000 && idade(agora, ultimoAviso) > 5000) {
            ultimoAviso = agora;
            serialPrintf("# [PPG] AVISO: sem batimentos ha mais de 5 s. Verifique dedo/ganho/ligacao do HB OUT.");
        }

        if (calibrando) {
            bool completo = (nCalib >= PPG_N_CALIB);
            bool timeout  = idade(agora, inicio) > PPG_TIMEOUT_CALIB_MS;

            if (completo || (timeout && nCalib >= PPG_MIN_CALIB)) {
                baseline = mediana(calib, nCalib);       // mediana, não média
                calibrando = false;
                serialPrintf("# [PPG] Baseline calibrado: %.1f BPM (%d amostras)", baseline, nCalib);
            } else if (timeout) {
                serialPrintf("# [PPG] Calibracao falhou (poucos batimentos). Reiniciando...");
                nCalib = 0;
                descartados = 0;
                inicio = agora;
            }
        }
    }
}

// ---------- Leitura do GSR com oversampling ----------
float lerGSR() {
    uint32_t soma = 0;
    for (int i = 0; i < 8; i++) soma += analogRead(GSR_PIN);
    float v = soma / 8.0f;
    return GSR_INVERTIDO ? (4095.0f - v) : v;
}

// ---------- Tarefa: GSR com baseline, nível tônico e picos fásicos ----------
void task_gsr(void *pvParameters) {
    TickType_t ultimoTick = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(GSR_PERIODO_MS);

    bool calibrando = true;
    bool iniciado = false;
    uint32_t inicio = millis();

    float rapido = 0.0f, lento = 0.0f;

    // Estatísticas da calibração (Welford)
    uint32_t n = 0;
    float mediaF = 0, m2F = 0;     // sinal suavizado (nível tônico)
    float mediaP = 0, m2P = 0;     // componente fásica (rapido - lento)

    float baseMedia = 0.0f;
    float baseStd = GSR_STD_MIN;
    float fasicoStd = GSR_STD_MIN;

    serialPrintf("# [GSR] Calibrando baseline (%d s). Fique parado e relaxe.", GSR_TEMPO_CALIB_MS / 1000);

    for (;;) {
        float raw = lerGSR();
        if (!iniciado) {
            rapido = lento = raw;
            iniciado = true;
        } else {
            rapido += GSR_ALPHA_RAPIDO * (raw - rapido);
            lento  += GSR_ALPHA_LENTO  * (raw - lento);
        }
        float fasico = rapido - lento;   // picos rápidos sobre a linha de base lenta
        uint32_t agora = millis();

        if (calibrando) {
            if (idade(agora, inicio) > GSR_IGNORAR_INICIO_MS) {
                n++;
                float dF = rapido - mediaF;
                mediaF += dF / n;
                m2F += dF * (rapido - mediaF);

                float dP = fasico - mediaP;
                mediaP += dP / n;
                m2P += dP * (fasico - mediaP);
            }
            if (idade(agora, inicio) > GSR_TEMPO_CALIB_MS && n > 10) {
                baseMedia = mediaF;
                baseStd   = fmaxf(sqrtf(m2F / n), GSR_STD_MIN);
                fasicoStd = fmaxf(sqrtf(m2P / n), GSR_STD_MIN);
                calibrando = false;
                serialPrintf("# [GSR] Baseline calibrado: media=%.1f std=%.1f ruido_fasico=%.1f",
                             baseMedia, baseStd, fasicoStd);
            }
        } else {
            GsrMsg m;
            m.tsMs    = agora;
            m.raw     = raw;
            m.fast    = rapido;
            m.zFasico = fasico / fasicoStd;                 // pico rápido vs ruído em repouso
            m.zTonico = (rapido - baseMedia) / baseStd;     // desvio vs estado de repouso
            xQueueSend(gsrQueue, &m, 0);
        }

        vTaskDelayUntil(&ultimoTick, periodo);   // período fixo de 50 ms
    }
}

// ---------- Tarefa: análise e decisão ----------
void task_analysis(void *pvParameters) {
    float bpmAtual = 0.0f, baseline = 0.0f;
    uint32_t tsPpg = 0, tsGsr = 0;
    bool ppgOk = false, gsrOk = false;

    uint32_t ultBpmAlto = 0, ultGsrAlto = 0;
    bool temBpmAlto = false, temGsrAlto = false;

    GsrMsg g = {};
    bool estresse = false;

    // Botão / perguntas
    bool btnAnterior = true;
    uint32_t tsBtn = 0;
    int pergunta = 0;
    bool janelaAtiva = false;
    uint32_t inicioJanela = 0;
    float picoGsrZ = 0.0f, picoBpmPerc = 0.0f;
    char evento[24] = "";

    uint32_t ultimoCsv = 0;

    serialPrintf("# tempo_ms,bpm,baseline_bpm,gsr_raw,gsr_fasico_z,gsr_tonico_z,estresse,estado,evento");

    for (;;) {
        // Esvazia as filas: sempre processa tudo e fica com o dado mais recente
        PpgMsg p;
        while (xQueueReceive(ppgQueue, &p, 0) == pdTRUE) {
            bpmAtual = p.bpm;
            baseline = p.baseline;
            tsPpg = p.tsMs;
            ppgOk = true;

            if (baseline > 0 && bpmAtual > baseline * (1.0f + BPM_AUMENTO_PERC / 100.0f)) {
                ultBpmAlto = p.tsMs;
                temBpmAlto = true;
            }
            if (janelaAtiva && baseline > 0) {
                float perc = (bpmAtual - baseline) / baseline * 100.0f;
                if (perc > picoBpmPerc) picoBpmPerc = perc;
            }
        }
        GsrMsg gm;
        while (xQueueReceive(gsrQueue, &gm, 0) == pdTRUE) {
            g = gm;
            tsGsr = gm.tsMs;
            gsrOk = true;

            if (gm.zFasico > GSR_K_FASICO || gm.zTonico > GSR_LIMIAR_Z_TONICO) {
                ultGsrAlto = gm.tsMs;
                temGsrAlto = true;
            }
            if (janelaAtiva && gm.zFasico > picoGsrZ) picoGsrZ = gm.zFasico;
        }

        uint32_t agora = millis();   // lido DEPOIS de esvaziar as filas

        // Frescor dos dados (descarta dados velhos / sensor solto)
        bool ppgFresco = ppgOk && idade(agora, tsPpg) < DADO_VELHO_MS;
        bool gsrFresco = gsrOk && idade(agora, tsGsr) < DADO_VELHO_MS;

        const char *estado;
        if (!ppgOk || !gsrOk) estado = "CALIBRANDO";
        else if (!ppgFresco)  estado = "SEM_PPG";
        else if (!gsrFresco)  estado = "SEM_GSR";
        else                  estado = "OK";
        bool dadosValidos = (estado[0] == 'O');

        // Janela temporal: ambos dispararam recentemente (não precisa ser no mesmo instante)
        bool bpmRecente = temBpmAlto && idade(agora, ultBpmAlto) < JANELA_COINCIDENCIA_MS;
        bool gsrRecente = temGsrAlto && idade(agora, ultGsrAlto) < JANELA_COINCIDENCIA_MS;
        estresse = dadosValidos && bpmRecente && gsrRecente;

        digitalWrite(LED_PIN, estresse ? HIGH : LOW);

        // ----- Botão: marca o início de uma pergunta (com debounce) -----
        bool btn = digitalRead(BTN_PIN);
        if (!btn && btnAnterior && idade(agora, tsBtn) > 250) {
            tsBtn = agora;
            pergunta++;
            janelaAtiva = true;
            inicioJanela = agora;
            picoGsrZ = 0.0f;
            picoBpmPerc = 0.0f;
            snprintf(evento, sizeof(evento), "PERGUNTA_%d", pergunta);
        }
        btnAnterior = btn;

        if (janelaAtiva && idade(agora, inicioJanela) > JANELA_PERGUNTA_MS) {
            janelaAtiva = false;
            serialPrintf("# Pergunta %d: pico GSR fasico = %.2f desvios | maior aumento de BPM = %.1f%%",
                         pergunta, picoGsrZ, picoBpmPerc);
        }

        // ----- CSV periódico (ou imediato ao apertar o botão) -----
        if (evento[0] || idade(agora, ultimoCsv) >= CSV_PERIODO_MS) {
            ultimoCsv = agora;
            serialPrintf("%lu,%.1f,%.1f,%.0f,%.2f,%.2f,%d,%s,%s",
                         (unsigned long)agora, bpmAtual, baseline, g.raw,
                         g.zFasico, g.zTonico, estresse ? 1 : 0, estado, evento);
            evento[0] = '\0';
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);
    delay(1000);

    serialMutex = xSemaphoreCreateMutex();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(HB_PIN, INPUT);

    // ADC do ESP32: 12 bits, 0-3,3 V (usado só pelo GSR)
    analogReadResolution(12);
    analogSetPinAttenuation(GSR_PIN, ADC_11db);

    beatQueue = xQueueCreate(8,  sizeof(BeatMsg));
    ppgQueue  = xQueueCreate(8,  sizeof(PpgMsg));
    gsrQueue  = xQueueCreate(16, sizeof(GsrMsg));

    if (!serialMutex || !beatQueue || !ppgQueue || !gsrQueue) {
        Serial.println("# Erro ao criar filas/mutex!");
        while (1) delay(1000);
    }

    // Só liga a interrupção depois que a fila existe
    attachInterrupt(digitalPinToInterrupt(HB_PIN), isr_batimento, RISING);

    xTaskCreatePinnedToCore(task_ppg,      "PPG Task",      4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(task_gsr,      "GSR Task",      4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_analysis, "Analysis Task", 4096, NULL, 1, NULL, 1);

    Serial.println("# Sistema iniciado. Aguardando calibracao...");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));   
}
