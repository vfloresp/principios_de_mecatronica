//Los estados de los semaforos son 0 en rojo 1 en amarillo y 2 en verde
int semaforo1=2;
int semaforo2=0;
int cont = 0;

void setup() {
  // put your setup code here, to run once:
  //Semaforo 1
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  //Semaforo 2
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  Serial.begin(9600);

  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  cli();
  TCCR1B = 0; TCCR1A = 0;
  TCCR1B |= (1 << CS12 | 1 << CS10);
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);
  sei();

  }

void loop() {
  // put your main code here, to run repeatedly:
  
}

ISR(TIMER1_OVF_vect)
{
  if(semaforo1 == 2 && semaforo2 == 0){
      if(cont < 7){
         TCNT1 = 0;
         cont = cont + 1;
      }else{
      semaforo1 = 1;
      digitalWrite(A2, LOW);
      digitalWrite(A1, HIGH);
      cont = 0;
    }
  }else{
    if(semaforo1 == 1 && semaforo2 == 0){
      if(cont == 0){
         TCNT1 = 470;
         cont = cont + 1;
      }else{
        TCNT1 = 0;
        semaforo1 = 0;
        semaforo2 = 2;
        digitalWrite(A1, LOW);
        digitalWrite(A0, HIGH);
        digitalWrite(A3, LOW);
        digitalWrite(A5, HIGH);
        cont = 0;
      }
    }else{
      if(semaforo1 == 0 && semaforo2 == 2){
         if(cont < 7){
            TCNT1 = 0;
            cont = cont + 1;
        }else{
          TCNT1 = 0;
          semaforo2 = 1;
          digitalWrite(A5, LOW);
          digitalWrite(A4, HIGH);
          cont = 0;
        }
      }else{
        if(semaforo1 == 0 && semaforo2 ==1){
          if(cont == 0){
            TCNT1 = 0;
            cont = cont + 1;
          }else{
            TCNT1 = 0;
            semaforo1 = 2;
            semaforo2 = 0;
            digitalWrite(A0, LOW);
            digitalWrite(A2, HIGH);
            digitalWrite(A4, LOW);
            digitalWrite(A3, HIGH);
            cont = 0;
          }
        }
      }
    }
  }

    
}
