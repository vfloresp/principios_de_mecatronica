void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  cli();
  TCCR1B = 0; TCCR1A = 0;
  TCCR1B |= (1 << CS12);
  TCNT1 = 3036;
  TIMSK1 |= (1 << TOIE1);
  sei();

  }

void loop() {
  // put your main code here, to run repeatedly:
  
}

ISR(TIMER1_OVF_vect)


    
}
