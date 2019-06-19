
//Testeando Motores con L293D
//Definimos pins
//Motor A
int enableA = 5;
int motorA1 = 6;
int motorA2 = 7;
//Motor B
int enableB = 8;
int motorB1 = 9;
int motorB2 = 10;
//sensores

//motor 1
int value0 = 0; 
int value1 = 0; 

//motor 2
int value2 = 0; 
int value3 = 0; 

int time0 = 0; 
//contadores
int cont0 = 0; 
int cont1 = 0; 
//direcciones
int dir1 = 1; 
int dir2 = 1; 

//errores
int error1= 0; 
int error2 =0; 

float sumerror1 =0; 
float sumerror2 = 0; 

float errorant1 = 0; 
float errorant2 = 0; 

// velocidad motores se mandan 
int velA= 0; 
int velB= 0; 


// salidas
float out1 = 0;
float out2 = 0;

float kp =.7; 
float ki =.5;
float kd =.1; 

float pid1 = 0; 
float pid2= 0; 


void setup() {

Serial.begin(9600); 
  //configuración
  pinMode (enableA, OUTPUT);
  pinMode (motorA1, OUTPUT);
  pinMode (motorA2, OUTPUT);  
  
  pinMode (enableB, OUTPUT);
  pinMode (motorB1, OUTPUT);
  pinMode (motorB2, OUTPUT); 

  pinMode (A0,INPUT); 
  pinMode (A1,INPUT); 
  
  digitalWrite (motorB1, 0);
  digitalWrite (motorB2, HIGH);
  digitalWrite (motorA1, 0);
  digitalWrite (motorA2, HIGH);
  digitalWrite (enableA, 0);
  digitalWrite (enableB, 0);


  time0 = millis(); 
  

}
void loop() {

   if (Serial.available()>0) 
   {
      
     
      String option = Serial.readString();
      int corte = option.indexOf('_');

      velA = option.substring(0,corte).toFloat();
      velB = option.substring(corte+1).toFloat();
      //velB = velB*1.1;
      int pwmOutputA = map(velA, 0, 40, 0 , 255); 
      analogWrite(enableA, pwmOutputA);
      int pwmOutputB = map(velB, 0, 40, 0 , 255); 
      analogWrite(enableB, pwmOutputB);

      Serial.println(option);
   }

 /*
   value1 = digitalRead(A0);
   value3 = digitalRead(A1); 

    //motor 1
   if (value1 == 1 && value0 == 0){
    cont0 ++; 
   }
   value0 = value1; 

   //motor2
   if (value3 == 1 && value2 == 0){
    cont1 ++; 
   }
   value2= value3; 
   
    int aux = 1;
   if(millis() >= 200){
    aux++;
    error1 = vel1 - (5)*cont0;
    error2 = vel2 - (5)*cont1;  

    sumerror1 += error1;
    sumerror2 += error2; 

    pid1 = error1*kp + (error1 - errorant1)*kd + sumerror1*ki; 
    pid2 = error2*kp + (error2 - errorant2)*kd + sumerror2*ki; 

    errorant1 = error1; 
    errorant2 = error2; 

    out1 = min(abs(pid1),255);
    out2 = min(abs(pid2),255);

    Serial.println("out1 = ");
     Serial.println(pid1);
    Serial.println("out2 = ");
    Serial.println(pid1);


    //motor 1
    analogWrite(enableB, out1);
    /*if (pid1>=0){
      //analogWrite(motorB1,0); 
      analogWrite(enableB, out1); 
      dir1 = 1;
    } else {
      analogWrite(motorB1,out1);
      analogWrite(motorB2,0); 
      dir1 = -1; 
    } 

    //motor 2
    analogWrite(enableA, out2); 
    if (pid2>=0){
      //analogWrite(motorA1,0); 
      analogWrite(enableA, out2); 
      dir1 = 1;
    } else {
      analogWrite(motorA1,out2);
      analogWrite(motorA2,0); 
      dir1 = -1; 
    } 
    delay(1000);
    cont0 = 0; 
    cont1 = 0;
    if(aux==3){
      sumerror1 = 0.00;
      sumerror2 = 0.00;
    }
    pid1=0.00;
    pid2=0.00;
    time0 = millis();  
   }
  */
 

  
   

}
