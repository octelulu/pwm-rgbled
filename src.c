#include "derivative.h"

int e; // eletrodo 0 ou eletrodo 1
char RX_data;
unsigned int red=0, green=0, blue=0;
unsigned int r_percentege=0, g_percentege=0, b_percentege=0;

int Baseline[2];
int Electrode[2]={9,10};
int Threshold[2]={100,100};
int TSICount[2];
int TSIDelta[2];
int Percentege = 0;

void init_UART(){
    
    SIM_SCGC4 |= (1<<10);            // Habilita clock da UART0
    SIM_SCGC5 |= (1<<9);             // Habilita clock GPIO do PortA (pinos 1 e 2)
    
    PORTA_PCR1 = 0x00000200;         // Configura pin MUX control para UART0 RX aparecer no pino 1 da PortA (ALT 2)
    PORTA_PCR2 = 0x00000200;         // Configura pin MUX control para UART0 TX aparecer no pino 2 da PortA (ALT 2)
    
    SIM_SOPT2 |= (1<<26)|(0<<27);    // Configura o sinal MCGFLLCLK de clock (bits 27 e 26: 01) para a UART0
    
    UART0_BDL = 68;                  // Configura o baud-rate para 19.200
    UART0_BDH = (0<<5);              // SBNS 0: 1 stop bit
    
    UART0_C2 = (1<<2)|(1<<3)|(1<<5); // Habilitação do receptor (RE), transmissor (TE) e receptor de interrupção (RIE)
    
    NVIC_ISER |= (1<<12);            // Habilita interrupção da UART0 (IRQ 12)
}

void UART0_IRQHandler(){
    
    RX_data = UART0_D;    // Leitura do caractere
    
    // Seleção das cores (RGB) e controle fino da intensidade
    switch(RX_data){
    case 'r':
        red=1;
        green=0;
        blue=0;
        Percentege=0;
        break;
    case 'g':
        red=0;
        green=1;
        blue=0;
        Percentege=0;
        break;
    case 'b':
        red=0;
        green=0;
        blue=1;
        Percentege=0;
        break;
    case '+':
        if(Percentege<100)
            Percentege+=1;
        break;
    case '-':
        if(Percentege>0)
            Percentege-=1;
        break;
    }
}

void init_TPM(){
    
    // Inicialização e configuração do PWM
    
    SIM_SCGC5 |= (1<<10)|(1<<12);     // Habilita clock GPIO do PortB e PortD
    SIM_SCGC6 |= (1<<24)|(1<<26);     // Habilita o clock do TPM0 e do TPM2

    SIM_SOPT2 |= (1<<24)|(0<<25);     // Configura o sinal MCGFLLCLK de clock (bits 25 e 24: 01) para o TPM

    PORTB_PCR18 = 0x00000300;         // Pin18 PortB: TPM2_CH0
    PORTB_PCR19 = 0x00000300;         // Pin19 PortB: TPM2_CH1
    PORTD_PCR1  = 0x00000400;         // Pin1 PortD:  TPM0_CH1
    
    // Prescale:0 CMOD:1 MOD:100000
    TPM0_SC |= (1<<3)|(1<<5);
    TPM2_SC |= (1<<3)|(1<<5);
    TPM0_MOD = 100000;
    TPM2_MOD = 100000;
    
    // Configuração TPM para modo PWM simétrico (Low-True Pulses e Center-Aligned PWM)
    TPM2_C0SC = 0b100100;
    TPM2_C1SC = 0b100100;
    TPM0_C1SC = 0b100100;
}

void init_TSI(){

    SIM_SCGC5 |= (1<<5);    // Habilita clock do TSI
    
    TSI0_GENCS |= (TSI_GENCS_MODE(0)		// Configura TSI para não detectar ruído
                   | TSI_GENCS_REFCHRG(4)	// Corrente referência: 8 uA
                   | TSI_GENCS_DVOLT(0)		// DV = 1.03 V, Vp = 1.33 V, Vm= 0.30 V
                   | TSI_GENCS_EXTCHRG(7)	// Corrente do eletrodo: 64 uA
                   | TSI_GENCS_PS(4)		// Preescale 16
                   | TSI_GENCS_NSCN(11)		// Número de scans por eletrodo: 12
                   | TSI_GENCS_TSIEN_MASK	// TSI ativado
                   | TSI_GENCS_STPE_MASK	// TSI ativado em modos de baixo consumo
                   );
    
    // Calibração
    
    TSI0_GENCS |= TSI_GENCS_EOSF_MASK;      			// Clear End of Scan Flag
   
    // Definição de counts sem toque no sensor
    for(e=0; e<2; e++){
        TSI0_DATA = (Electrode[e]<<TSI_DATA_TSICH_SHIFT);	// Seleciona o eletrodo
        TSI0_DATA |= TSI_DATA_SWTS_MASK;			// Software Trigger
        while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK));		// Espera o fim do scan
        Baseline[e] = (TSI0_DATA & TSI_DATA_TSICNT_MASK);	// Define o numero de counts referência
        TSI0_GENCS |= TSI_GENCS_EOSF_MASK;			// Clear End of Scan Flag
    }
}
    
void SliderTrigger(){
    
    int delta;
    
    for(e=0; e<2; e++){
        TSI0_DATA = (Electrode[e]<<TSI_DATA_TSICH_SHIFT);	// Seleciona o eletrodo
        TSI0_DATA |= TSI_DATA_SWTS_MASK;			// Software Trigger
        while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK));		// Espera o fim do scan
        TSICount[e] = (TSI0_DATA & TSI_DATA_TSICNT_MASK);	// Salva o valor do scan
        TSI0_GENCS |= TSI_GENCS_EOSF_MASK;			// Clear End of Scan Flag
        delta = TSICount[e] - Baseline[e];			// Calcula a diferença de counts entre o scan e a referência
        if(delta < 0)                        			// Tratamento do valor de delta
            TSIDelta[e] = 0;
        else
            TSIDelta[e] = delta;
    }
}

void SliderRead(){
    
    // Calcula o ponto exato do toque em porcentagem
    if((TSIDelta[0] > Threshold[0])||(TSIDelta[1] > Threshold[1])){
        Percentege = (TSIDelta[1]*100)/(TSIDelta[0]+TSIDelta[1]);
    }
}

void intascii(int n, int i, char *s) {
    int k=n;
    s[n]='\0';
    for (; n!=0; n-- ){
        s[n-1]='0';
    }
    for (; k!=0; k--) {
        s[k-1]=i%10 + '0';
        i=i/10;
    }
}

void putchar_UART(char c){
    while(!(UART0_S1 & UART0_S1_TDRE_MASK)); // se não está vazio (Register Empty), espera
    UART0_D = c; // envia caracter a ser transmitido
}

void print_UART(char *s){ // imprime string na UART
    int i=0;
    while(s[i] != 0) {
        putchar_UART(s[i]);
        i++;
    }
}

void print(){
    
    char r[4], g[4], b[4], c1[6], c2[6], d1[6], d2[6], b1[6], b2[6];
    
    // Conversão
    intascii(3, r_percentege, r);
    intascii(3, g_percentege, g);
    intascii(3, b_percentege, b);
    intascii(5, TSICount[0], c1);
    intascii(5, TSICount[1], c2);
    intascii(5, TSIDelta[0], d1);
    intascii(5, TSIDelta[1], d2);
    intascii(5, Baseline[0], b1);
    intascii(5, Baseline[1], b2);
   
    print_UART("Red:");
    print_UART(r);
    print_UART("% Green:");
    print_UART(g);
    print_UART("% Blue:");
    print_UART(b);
    
    print_UART("% | Count e1:");
    print_UART(c1);
    
    print_UART(" Base e1:");
    print_UART(b1);
    
    print_UART(" Delta e1:");
    print_UART(d1);
          
    print_UART(" | Count e2:");
    print_UART(c2);
    
    print_UART(" Base e2:");
    print_UART(b2);
    
    print_UART(" Delta e2:");
    print_UART(d2);
    
    putchar_UART('\n'); // Pula uma linha
    putchar_UART('\r'); // Retorna o cursor para o início na linha
}

int main(void){
    
    init_UART();
    init_TPM();
    init_TSI();
    
    TPM2_C0V=0;
    TPM2_C1V=0;
    TPM0_C1V=0;
    
    for(;;){
        
        SliderTrigger();
        SliderRead();
        
        // Controle da intensidade de RGB do led através de PWM
        if(red){
            r_percentege = Percentege;
            TPM2_C0V = (TPM2_MOD*r_percentege)/100;
        }
        if(green){
            g_percentege = Percentege;
            TPM2_C1V = (TPM2_MOD*g_percentege)/100;
        }
        if(blue){
            b_percentege = Percentege;
            TPM0_C1V = (TPM0_MOD*b_percentege)/100;
        }
        
        print();
    }
}
