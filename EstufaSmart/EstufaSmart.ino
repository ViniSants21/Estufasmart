#include <WiFiS3.h>
#include <Arduino_LED_Matrix.h>
#include <DHT.h>
#include "arduino_secrets.h"
#include <math.h>
#include "estados.h"

const uint8_t DHT_PIN=2, JANELA_PIN=3, BOTAO_PIN=4;
const uint8_t VERDE=5, AMARELO=6, VERMELHO=7, BUZZER=8, RELE=9;
// Ajuste conforme o modulo: muitos reles ligam com LOW.
const uint8_t RELE_LIGADO=LOW;
const bool BUZZER_ATIVO=true; // false para buzzer passivo
const uint8_t JANELA_ABERTA=HIGH; // reed entre D3 e GND: aberto = HIGH
const int LDR_ESCURO=0, LDR_CLARO=1023; // calibrar com o Monitor Serial
const float TEMP_MIN=20, TEMP_MAX=28, MARGEM=1;
const int LUZ_ALTA=85;
const uint32_t LEITURA_MS=5000, FORA_MS=30000, JANELA_ATENCAO_MS=20000, JANELA_CRITICA_MS=60000;
const char *DEVICE_ID="EST-SALA01";
Estado estado=FALHA_SENSOR;
const char *nomes[]={"NORMAL","ATENCAO","CRITICO","RECONHECIDO","FALHA_SENSOR"};
DHT dht(DHT_PIN,DHT11);
ArduinoLEDMatrix matrix;
WiFiClient client;
uint8_t bitmap[8][12]={0};
const uint8_t letras[5][7]={
 {17,25,25,21,19,19,17}, // N
 {14,17,17,31,17,17,17}, // A
 {14,17,16,16,16,17,14}, // C
 {30,17,17,30,20,18,17}, // R
 {31,16,16,30,16,16,16}  // F
};
float temp=NAN, umid=NAN;
int luz=0;
bool leituraValida=false, janelaAberta=false, janelaContando=false, foraContando=false, reconhecido=false;
uint8_t falhas=0;
uint32_t leituraEm=0, janelaDesde=0, foraDesde=0, seq=0, bipeEm=0;
bool botaoRaw=HIGH, botaoEstavel=HIGH;
uint32_t botaoMudou=0;
struct Amostra {uint32_t seq; float temp,umid; int luz; bool janela,valida; Estado estado;};
const int CAPACIDADE=40;
Amostra fila[CAPACIDADE];
int inicio=0, quantidade=0;
uint32_t perdidas=0, wifiTentativa=0, envioTentativa=0, respostaDesde=0;
bool esperando=false;
char resposta[100]; size_t respostaLen=0;

void mostrarEstado() {
 memset(bitmap,0,sizeof(bitmap));
 for(int y=0;y<7;y++) for(int x=0;x<5;x++) bitmap[y][x+3]=(letras[estado][y]>>(4-x))&1;
 matrix.renderBitmap(bitmap,8,12);
}
void mudarEstado(Estado novo) {
 if(novo==estado) return;
 estado=novo; bipeEm=millis(); mostrarEstado();
 Serial.print("Estado: "); Serial.println(nomes[estado]);
}
void lerSensores(uint32_t agora) {
 temp=dht.readTemperature(); umid=dht.readHumidity();
 leituraValida=isfinite(temp)&&isfinite(umid);
 if(leituraValida) falhas=0;
 else {if(falhas<3) falhas++; foraContando=false;}
 int raw=analogRead(A0);
 luz=constrain(map(raw,LDR_ESCURO,LDR_CLARO,0,100),0,100);
 if(leituraValida) {
  bool fora=temp<TEMP_MIN||temp>TEMP_MAX;
  if(fora&&!foraContando) {foraContando=true; foraDesde=agora;}
  if(!fora) foraContando=false;
 }
 Serial.print("T=");Serial.print(temp);Serial.print(" U=");Serial.print(umid);
 Serial.print(" LDR=");Serial.print(raw);Serial.print(" Luz%=");Serial.println(luz);
}
void entradas(uint32_t agora) {
 janelaAberta=digitalRead(JANELA_PIN)==JANELA_ABERTA;
 if(janelaAberta&&!janelaContando){janelaContando=true;janelaDesde=agora;}
 if(!janelaAberta)janelaContando=false;
 bool raw=digitalRead(BOTAO_PIN);
 if(raw!=botaoRaw){botaoRaw=raw;botaoMudou=agora;}
 if(agora-botaoMudou>=40&&raw!=botaoEstavel){
  botaoEstavel=raw;
  if(raw==LOW&&estado==CRITICO)reconhecido=true;
 }
}
void atualizarEstado(uint32_t agora) {
 if(falhas>=3){mudarEstado(FALHA_SENSOR);return;}
 // Durante falhas isoladas, nao declarar normal usando uma leitura antiga.
 if(!leituraValida)return;
 bool critica=(foraContando&&agora-foraDesde>FORA_MS)||
              (janelaContando&&agora-janelaDesde>JANELA_CRITICA_MS);
 bool atencao=temp<=TEMP_MIN+MARGEM||temp>=TEMP_MAX-MARGEM||luz>=LUZ_ALTA||
              (janelaContando&&agora-janelaDesde>JANELA_ATENCAO_MS);
 bool normalizou=temp>=TEMP_MIN&&temp<=TEMP_MAX&&!janelaAberta&&luz<LUZ_ALTA;
 if(reconhecido&&normalizou)reconhecido=false;
 if(reconhecido)mudarEstado(RECONHECIDO);
 else if(critica)mudarEstado(CRITICO);
 else if(atencao||foraContando||janelaAberta)mudarEstado(ATENCAO);
 else mudarEstado(NORMAL);
}
void saidas(uint32_t agora) {
 digitalWrite(VERDE,estado==NORMAL);
 digitalWrite(AMARELO,estado==ATENCAO||estado==FALHA_SENSOR);
 digitalWrite(VERMELHO,estado==CRITICO||estado==RECONHECIDO||estado==FALHA_SENSOR);
 // Reconhecer o alarme silencia o som, mas preserva a ventilacao.
 bool ventilar=estado==CRITICO||estado==RECONHECIDO||estado==FALHA_SENSOR;
 digitalWrite(RELE,ventilar?RELE_LIGADO:(RELE_LIGADO==LOW?HIGH:LOW));
 bool som=estado==CRITICO;
 if(estado==ATENCAO){
  if(agora-bipeEm>=10000)bipeEm=agora;
  som=agora-bipeEm<180;
 }
 if(BUZZER_ATIVO)digitalWrite(BUZZER,som?HIGH:LOW);
 else {if(som)tone(BUZZER,2000);else noTone(BUZZER);}
}
void guardar() {
 if(quantidade==CAPACIDADE){inicio=(inicio+1)%CAPACIDADE;quantidade--;perdidas++;}
 int p=(inicio+quantidade)%CAPACIDADE;
 fila[p]={seq++,temp,umid,luz,janelaAberta,leituraValida,estado};quantidade++;
 Serial.print("Fila: ");Serial.print(quantidade);Serial.print(" Perdidas: ");Serial.println(perdidas);
}
void finalizarEnvio(bool aceito) {
 client.stop();esperando=false;
 if(aceito&&quantidade){inicio=(inicio+1)%CAPACIDADE;quantidade--;}
}
void rede(uint32_t agora) {
 if(!ENABLE_NETWORK)return;
 if(WiFi.status()!=WL_CONNECTED){
  if(esperando)finalizarEnvio(false);
  if(agora-wifiTentativa>=15000){wifiTentativa=agora;WiFi.begin(SECRET_SSID,SECRET_PASS);}
  return;
 }
 if(esperando){
  while(client.available()&&respostaLen<sizeof(resposta)-1){
   char c=client.read();resposta[respostaLen++]=c;
   if(c=='\n'){
    resposta[respostaLen]=0;
    int codigo=0;sscanf(resposta,"HTTP/%*s %d",&codigo);
    // 422 descarta amostra invalida (ex.: falha do sensor); nao trava a fila.
    finalizarEnvio((codigo>=200&&codigo<300)||codigo==422);return;
   }
  }
  if(!client.connected()||agora-respostaDesde>2000||respostaLen>=sizeof(resposta)-1)finalizarEnvio(false);
  return;
 }
 if(!quantidade||agora-envioTentativa<1000)return;
 envioTentativa=agora;
 Amostra a=fila[inicio];
 char t[24],u[24],body[320];
 if(a.valida){snprintf(t,sizeof(t),"%.1f",a.temp);snprintf(u,sizeof(u),"%.1f",a.umid);}
 else {strcpy(t,"null");strcpy(u,"null");}
 snprintf(body,sizeof(body),"{\"device_id\":\"%s\",\"placa\":\"UNO_R4_WIFI\",\"seq\":%lu,\"temp\":%s,\"umid\":%s,\"janela\":\"%s\",\"luz_pct\":%d,\"estado\":\"%s\"}",DEVICE_ID,(unsigned long)a.seq,t,u,a.janela?"ABERTA":"FECHADA",a.luz,nomes[a.estado]);
 if(!client.connect(SERVER_HOST,SERVER_PORT)){client.stop();return;}
 client.print("POST /api/telemetria HTTP/1.1\r\nHost: ");client.print(SERVER_HOST);
 client.print("\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: ");
 client.print(strlen(body));client.print("\r\n\r\n");client.print(body);
 esperando=true;respostaDesde=millis();respostaLen=0;
}
void setup() {
 Serial.begin(115200); // Nao espera o computador: funciona alimentado por USB/fonte.
 pinMode(JANELA_PIN,INPUT_PULLUP);pinMode(BOTAO_PIN,INPUT_PULLUP);
 for(uint8_t p=VERDE;p<=RELE;p++){
  digitalWrite(p,p==RELE?(RELE_LIGADO==LOW?HIGH:LOW):LOW);pinMode(p,OUTPUT);
 }
 analogReadResolution(10);dht.begin();matrix.begin();mostrarEstado();
 // Primeira leitura imediata, sem delay no loop.
 leituraEm=millis()-LEITURA_MS;wifiTentativa=millis()-15000;
}
void loop() {
 uint32_t agora=millis();
 entradas(agora);
 bool coletar=agora-leituraEm>=LEITURA_MS;
 if(coletar){leituraEm=agora;lerSensores(agora);}
 atualizarEstado(agora);saidas(agora);
 if(coletar)guardar();
 rede(agora);
}
