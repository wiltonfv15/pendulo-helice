//---------------------------------------------------------
// WILTON VASCONCELOS  
// CONTROLE PID PARA SISTEMA PÊNDULO-HELICE
//---------------------------------------------------------
//  sensor offset = A0
//  sensor de angulo = A1
//---------------------------------------------------------

#include <Servo.h>//Usando a biblioteca Servo para controle do motor
Servo esc; 

// --------------Configurações e Pinagem--------------------
void setup()
{
  Serial.begin(9600);
  esc.attach(8); // Conecta o motor no pino 8
  esc.writeMicroseconds(1000); //Inicia o motor no valor 1000
}
// ------------------Variaveis de controle------------------

const int minPwm = 1015; //valor de PWM que mantém o pêndulo na posição atual
bool comando = false; //define se o PID deve atuar
double setpoint = 0, setpointUsuario = 0, setPointGrau = 0; //definem a posição escolhida pelo usuário (setpoint)
double error = 0; //erro atual em relação ao setpoint
double angulo = 0, lastAngle = 0; //ângulo atual e o último medido
double kp = 0.4, ki = 0.08, kd = 0.3; //constantes PID
double P = 0, I = 0, D = 0, PID = 0; //variáveis auxiliares PID
double controlePWM = 0; //valor do PWM enviado ao motor
//utilizado no cálculo do tempo decorrido entre cada aplicação do PID
long lastProcess = 0; 
float deltaTime = 0; 
//leitura de informações recebidas via porta serial
int leitura_serial = -1; 
char str_set_point[4];
char str_P[7];   
char str_I[7];   
char str_D[7];
// ---------------------------------------------------------

double varAnalog=0;

void modo_pid()
{
	while (true) 
	{
		if (Serial.available() != 0) {
			leitura_serial = Serial.read();
			//I - Iniciar
			if (leitura_serial == 73) {
			  comando = true;
			  esc.writeMicroseconds(1000); //Inicia o esc no valor 1000
			  for (int i = 0; i < 3000; i++) {
				  analogRead(A0);
				  delay(1);
			  }
			}
			//P - Parar
			if (leitura_serial == 80){

       if(controlePWM>1060){
        
        for(int i = 1060; i>1000; i=i-1){  // desligamento suave;
        esc.writeMicroseconds(i);
        delay(70);
        }
       }   
			 setpointUsuario = 0;
       setPointGrau = 0;
       setpoint = 0;
       PID = 0;
       controlePWM = 0;
			 comando = false;     
			}
			//$ - Sair
			if (leitura_serial == 36) {
			  setpointUsuario = 0;
        //setpoint = 0;
			  break; 
			}
			//L - Leitura
			if (leitura_serial == 76) {
			  Serial.print("L,");
			  Serial.print(controlePWM);   
			  Serial.print(",");
			  Serial.print(angulo); 
			  Serial.print(",");
			  Serial.println(error);         
			} 
			//S - Atribuição do SetPoint       
			else if (leitura_serial == 83) {         
				//Aguarda Recebimento dos Bytes do valor de SetPoint         
				while (Serial.available()<2){};                  
				str_set_point[0] = Serial.read();         
				str_set_point[1] = Serial.read();         
				str_set_point[2] = '/n';                  
				double set_point_temp = setPointGrau;         
				setPointGrau = atof(str_set_point);     
                             
				//Garante que o setpoint esteja entre 0º e 90º
				if (setPointGrau > 90 || setPointGrau < 0) {          
					setPointGrau = set_point_temp; 
				}			
				//Mapeia o grau escolhido para o valor análogo à leitura do potenciômetro de controle manual 
				//90º <-> 1023
				setpointUsuario = map(setPointGrau, 0,90,0,1023);
			}   
			//G - Atribuição dos Ganhos Kp - Ki - Kd       
			else if (leitura_serial == 71) {
				//Aguarda Recebimento dos Bytes dos Valores do Ganhos         
				while (Serial.available()<18){};                  
				
				str_P[0] = Serial.read();         
				str_P[1] = Serial.read(); 
				str_P[2] = Serial.read();         
				str_P[3] = Serial.read();         
				str_P[4] = Serial.read();         
				str_P[5] = Serial.read();         
				str_P[6] = '\n';                  
				
				str_I[0] = Serial.read();         
				str_I[1] = Serial.read();         
				str_I[2] = Serial.read();         
				str_I[3] = Serial.read();         
				str_I[4] = Serial.read();         
				str_I[5] = Serial.read();         
				str_I[6] = '\n';                  
				
				str_D[0] = Serial.read();         
				str_D[1] = Serial.read();         
				str_D[2] = Serial.read();         
				str_D[3] = Serial.read();         
				str_D[4] = Serial.read();         
				str_D[5] = Serial.read();         
				str_D[6] = '\n';   
				
				//Limita as constantes entre 0 e 10
				kp = atof(str_P) > 00.000 && atof(str_P) <= 10.000 ? atof(str_P) : kp;         
				ki = atof(str_I) > 00.000 && atof(str_I) <= 10.000 ? atof(str_I) : ki;         
				kd = atof(str_D) > 00.000 && atof(str_D) <= 10.000 ? atof(str_D) : kd;
				
				Serial.println(kp);        
				Serial.println(ki);        
				Serial.println(kd);
			}
			  leitura_serial = -1;
		  }
		  //Lê a variável comando que define se o sistema de controle deve agir
		  if (comando) {
			setpoint = map(setpointUsuario, 0, 1023, minPwm, 1216); //Encontra o setpoint para o valor do PWM
			angulo = analogRead(A1); //Lê o potenciometro de angulo no Pino A1
			angulo = map(angulo, 0, 100, 0, 30); //Mapeia o Potenciometro de angulo
			
			//Cálculo do tempo do loop para as parcelas I e D
			//Diminui o erro para I e D devido a algum delay de processamento do arduino
			deltaTime = (millis() - lastProcess)/1000.0; 
			lastProcess = millis() ;
			
			// -------------------CONTROLADOR PID ----------------------
      setPointGrau = (setpoint - minPwm) * 0.45;
			error = (setPointGrau - angulo);  
			
			//Proporcional
			P  = error * kp;
			
			//Integral
			//I = I + (error * ki) * deltaTime;
			I = I + (error * ki);
			
			//Derivativo
			D = (lastAngle - angulo) * kd / deltaTime;
			lastAngle = angulo;
			
			PID = P + I + D;    // se PID = 0  ==> Perfeito OK
			
			// converte p/ controle
			controlePWM = PID + minPwm;
			
			//Limita o PWM para evitar força excessiva
			if (controlePWM < 0)
			  controlePWM = 0;
			else if (controlePWM > 1216)
			  controlePWM = 1216;
			
			esc.writeMicroseconds(controlePWM); //Envia o valor do PWM ao motor 
			delay(50); //Incluído delay para evitar divisão por zero (deltaTime = 0) no cálculo da componente derivativa 
		  }
	}
}

void modo_manual()
{
	while (true) 
	{
		if (Serial.available() != 0) {
			leitura_serial = Serial.read();
			//I - Iniciar
			if (leitura_serial == 73) {
			  comando = true;
			  esc.writeMicroseconds(1000); //Inicia o esc no valor 1000
			  for (int i = 0; i < 3000; i++) {
				  analogRead(A0);
				  delay(1);
			  }
			}
			//P - Parar
			if (leitura_serial == 80) {
			  esc.writeMicroseconds(0);
			  setpointUsuario = 0; //evita solavanco ao reiniciar a operação
			  comando = false;
			}
			//$ - Sair
			if (leitura_serial == 36) {
			  setpointUsuario = 0; //evita solavanco ao reiniciar a operação
			  break; 
			}
			//L - Leitura
			if (leitura_serial == 76) {
			  Serial.print("L,");
			  Serial.print(controlePWM);   
			  Serial.print(",");
			  Serial.print(angulo); 
			  Serial.print(",");
			  Serial.println(error);         
			} 
			leitura_serial = -1;
		  }
		  setpoint = analogRead(A0); //Lê o potenciometro de Setpoint no Pino A0
		  //Lê a variável comando que define se o sistema de controle deve agir
		  if (comando) {
			varAnalog = map(setpoint, 0, 1023, 1005, 1150); //Encontra o setpoint para o valor do PWM
			angulo = analogRead(A1); //Lê o potenciometro de angulo no Pino A1
			angulo = map(angulo, 0, 100, 0, 30); //Mapeia o Potenciometro de angulo

      
      
			//Cálculo do tempo do loop para as parcelas I e D
			//Diminui o erro para I e D devido a algum delay de processamento do arduino
			deltaTime = (millis() - lastProcess)/1000.0; 
			lastProcess = millis() ;
			
			// -------------------MODO MANUAL ----------------------
			setPointGrau = (setpoint - minPwm) * 0.45; //Calcula o grau desejado a partir do setpoint de PWM/leitura do potênciometro
			error = (setPointGrau - angulo);  
			
			//Proporcional
			P  = error * kp;
			
			//Integral
			//I = I + (error * ki) * deltaTime;
			I = I + (error * ki);
			
			//Derivativo
			D = (lastAngle - angulo) * kd / deltaTime;
			lastAngle = angulo;
			
			PID = P + I + D;    // se PID = 0  ==> Perfeito OK
			
			// converte p/ controle
			controlePWM = PID + minPwm;
			
			//Limita o PWM para evitar força excessiva
			if (controlePWM < 0)
			  controlePWM = 0;
			else if (controlePWM > 1216)
			  controlePWM = 1216;
			
			//esc.writeMicroseconds(controlePWM); //Envia o valor do PWM ao motor 
      esc.writeMicroseconds(varAnalog);
			delay(10); //Incluído delay para evitar divisão por zero (deltaTime = 0) no cálculo da componente derivativa 
		  }
	}
}

void loop()
{
	while (Serial.available()==0){}; 
	
	leitura_serial = Serial.read();
	//W - MODO MANUAL
	if (leitura_serial == 87) {
	  Serial.println('W');
	  modo_manual();
	}
	//X - MODO PID
	if (leitura_serial == 88) {
	  Serial.println('X');
    modo_pid();
	}
}
