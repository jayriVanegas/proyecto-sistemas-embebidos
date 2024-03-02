#include "mbed.h"
#include <cstdint>
#include <cmath>
#include <numeric>
#include <vector>
#include"Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include <string>

#define SDA p28 // Pin SDA OLED
#define SCL p27 // Pin SCL OLED
#define N 100
#define M_PI 3.14159265
DigitalOut led1(LED1);
AnalogIn curIn(p15);
AnalogIn vin(p19);
volatile uint16_t carr[N];
volatile uint16_t varr[N];
volatile bool readyf = false; 

const double PI = 3.14159265358979323846;
Ticker t;

class I2CPreInit : public I2C
{
public:
    I2CPreInit(PinName sda, PinName scl) : I2C(sda, scl)
    {
        frequency(400000);
        start();
    };
};
I2CPreInit oledI2C(SDA, SCL);
Adafruit_SSD1306_I2c myOled(oledI2C,p11,0x78,64,128);
void sample() {
    static uint16_t i = 0;
    if (i < N && !readyf) { 
        carr[i] = curIn.read_u16();
        varr[i] = vin.read_u16();
        i++;
    } else if (!readyf) {
        readyf = true;
        i = 0;
    }
}


void filter(const volatile uint16_t uvol[], float result[], int size);
void applyMovingAverageFilter(float* input, float* output, int length, int windowSize);
void desplazar(float senal[],  float& desplazamiento_y);
void encontrar_min_max(float señal[], int longitud, int16_t minimos[], int16_t maximos[], int *num_minimos, int *num_maximos);
pair<float, float> encontrarExtremos(float* senal, int tamano);
float calcularDesfase(float* señal1, float* señal2, int longitud);
void mostrar(float corriente[],float voltaje[]);
void impcadena(const char *str, int x, int y, uint16_t color, uint16_t size);
int main() {
    t.attach(sample, 167us);
    printf("This is the bare metal blinky example running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    while (true) {
        if (readyf) {
            t.detach();
            float cfil[N];
            float vfil[N];
            float voltaje[N];
            float corriente[N];
            int i,j;
            float ang;
            float a,b,p,s,q,fp;

            filter(varr, vfil, N);
            filter(carr, cfil, N);

            //Aplicar el filtro de media móvil a las señales
            applyMovingAverageFilter(cfil,corriente, N, 5); // Tamaño de la ventana del filtro: 5
            applyMovingAverageFilter(vfil,voltaje , N, 5);

            mostrar(corriente,voltaje);

            //for (int i = 0; i < N; i++) {  
            //   printf("%f,%f\n",voltaje[i],corriente[i]);
            //}

            readyf = false;
            t.attach(sample, 167us);
        }
    }
}
void filter(const volatile uint16_t uvol[], float result[], int size) {
    // Coeficientes del filtro (Filtro de segundo orden Butterworth)
    double w2 = 50.0; // Frecuencia de corte del filtro pasabajas

    float b1=1.867, b2=-0.8752;
    float a1=-0.004247, a2=-0.004063;

    // Aplicar el filtro
    result[0] = uvol[0];
    result[1] = uvol[1];
    for (int k = 2; k < size; k++) {
        result[k] = b1 * uvol[k - 1] + b2 * uvol[k - 2] - a1 * result[k - 1] - a2 * result[k - 2];
    }
}

// Función para suavizado de la señal
void applyMovingAverageFilter(float* input, float* output, int length, int windowSize) {
    for (int i = 0; i < length; i++) {
        float sum = 0.0;
        int count = 0;
        for (int j = i; j > i - windowSize && j >= 0; j--) {
            sum += input[j];
            count++;
        }
        output[i] = sum / count;
    }
}

pair<float, float> encontrarExtremos(float senal[], int tamano) {

    float maximo = senal[0]; // Inicializamos el máximo con el primer elemento
    float minimo = senal[0]; // Inicializamos el mínimo con el primer elemento
    
    // Recorremos la señal para encontrar el máximo y el mínimo
    for (int i = 1; i < tamano; ++i) {
        if (senal[i] > maximo) {
            maximo = senal[i];
        }
        if (senal[i] < minimo) {
            minimo = senal[i];
        }
    }   
    return {maximo, minimo};
}

float calcularDesfase(float* señal1, float* señal2, int longitud) {
    //metodo de correlacion cruzada
    double maxCorrelacion = -1.0; // Inicializar la correlación máxima a un valor mínimo
    int mejorDesfase = 0;

    // Iterar sobre los posibles desfases
    for (int desfase = 0; desfase < longitud; ++desfase) {
        double correlacion = 0.0;

        // Calcula la correlación entre las señales con el desfase actual
        for (int i = 0; i < longitud; ++i) {
            correlacion += señal1[i] * señal2[(i + desfase) % longitud];
        }

        // Actualizar el máximo de correlación y el mejor desfase
        if (correlacion > maxCorrelacion) {
            maxCorrelacion = correlacion;
            mejorDesfase = desfase;
        }
    }

    return mejorDesfase;
}

void mostrar(float corriente[],float voltaje[]){
    // Calcular los valores RMS (valor eficaz)
    pair<float, float> valores_v = encontrarExtremos(voltaje, N);
    float v_peak = ((( valores_v.first-valores_v.second)/ 2.0)*3.3/65535)*808.12;
    pair<float, float> valores_i = encontrarExtremos(corriente, N);
    float i_peak = ((( valores_i.first-valores_i.second) / 2.0)*3.3/65535)/0.185;
    float i_rms = i_peak / sqrt(2.0); // RMS
    float v_rms = v_peak / sqrt(2.0); // RMS
    float desfase = calcularDesfase(voltaje,corriente,N);
    float FP = cos(desfase*PI/180); //Factor de potencia
    float potAparente = v_rms*i_rms;// Potencia aparente
    float potReal = potAparente*FP ;// Potencia Real
    float potReac = sqrt(potAparente*potAparente-potReal*potReal);
    char i_peak_str[20];
    char v_peak_str[20];
    char v_rms_str[20];
    char i_rms_str[20];
    char potAparente_str[20];
    char potReal_str[20];
    char desfase_str[20];
    char potReac_str[20];
    char fp_str[20];
    char str[20] = "ip= ";
    char str1[20] = "vp= ";
    char str2[20] = "vrms=";
    char str3[20] = "irms=";
    char str4[20] = "s= ";
    char str5[20] = "p= ";
    char str6[20] = "Q= ";
    char str7[20] = "des= ";
    char str8[20] = "fp= ";
    sprintf(i_peak_str, "%s%.2f",str,i_peak);
    sprintf(v_peak_str, "%s%.2f",str1, v_peak);
    sprintf(v_rms_str, "%s%.2f",str2, v_rms);
    sprintf(i_rms_str, "%s%.2f",str3, i_rms);
    sprintf(potAparente_str, "%s%.2f",str4, potAparente);
    sprintf(potReal_str, "%s%.2f",str5, potReal);
    sprintf(potReac_str, "%s%.2f",str6, potReac);
    sprintf(desfase_str, "%s%.2f",str7, desfase);
    sprintf(fp_str, "%s%.2f",str8, FP);
    char resultado[200];
    char resultado1[200];  // Asegúrate de tener suficiente espacio
    sprintf(resultado, "%s\n%s\n%s\n%s\n",
        i_peak_str, v_peak_str, v_rms_str, i_rms_str);
    sprintf(resultado1, "%s\n%s\n%s\n%s\n%s\n",potAparente_str, potReal_str, potReac_str, desfase_str, fp_str);
    // Mostrar todas las líneas al mismo tiempo
    impcadena(resultado, 0, 1, 1, 2);
    impcadena(resultado1, 0, 1, 1, 2);
    //printf("Voltaje pico: %.4f.\n",v_peak);
    //printf("Corriente pico: %.4f.\n",i_peak);
    //printf("Voltaje RMS: %s\n",v_rms_str);
    //printf("Corriente RMS: %.4f.\n",i_rms);
    //printf("El desfase es: %.2f radianes.\n", desfase);
    //printf("El factor de potencia es: %.4f.\n",FP);
    //printf("La portencia aparente es: %.4f.\n",potAparente);
    //printf("La portencia real es: %.4f.\n",potReal);
    //printf("La portencia reactiva es: %.4f.\n",potReac);
     //printf("%f,%f,%f,%f,%f,%f,%f,%f\n",v_peak,i_peak,v_rms,i_rms,desfase,FP,potAparente,potReal);
    //printf("\n");
    //printf("\n");
}
void impcadena(const char *str, int x, int y, uint16_t color, uint16_t size){
        myOled.setTextCursor(x, y); //Acomodo la ubicacion del cursor
        myOled.setTextColor(color); //Acomodo el color de la letra
        myOled.setTextSize(size); //Acomoda el tamaño de la letra
        myOled.clearDisplay();
        myOled.display();
        while (*str != '\0') //Recorre todos los valores de la cadena hasta que un caracter sea nulo
        {
            
            myOled.writeChar(*str);
            str++; //Contador de caracteres
        }
    
        myOled.display(); //Actualiza el display
        //wait_ns(1000000000);
    }