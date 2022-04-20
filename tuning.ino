int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'b':
      if (cmd == '+')    K1X += 0.02;
      if (cmd == '-')    K1X -= 0.02;
      printValues();
      break;
    case 'g':
      if (cmd == '+')    K2X += 0.01;
      if (cmd == '-')    K2X -= 0.01;
      printValues();
      break;  
    case 'c':
      if (cmd == '+')    K3X += 0.005;
      if (cmd == '-')    K3X -= 0.005;
      printValues();
      break;  

    case 'p':
      if (cmd == '+')    K1Y += 0.02;
      if (cmd == '-')    K1Y -= 0.02;
      printValues();
      break;
    case 'd':
      if (cmd == '+')    K2Y += 0.01;
      if (cmd == '-')    K2Y -= 0.01;
      printValues();
      break;
    case 's':
      if (cmd == '+')    K3Y += 0.005;
      if (cmd == '-')    K3Y -= 0.005;
      printValues();
      break;  
   case 't':
      if (cmd == '+')    test = true;
      if (cmd == '-')    test = false;
      if (test) Serial.println("test ON");
      else Serial.println("test OFF");
      break;                    
   }
}

void printValues() {
  Serial.print("K1X: "); Serial.print(K1X);
  Serial.print(" K2X: "); Serial.print(K2X);
  Serial.print(" K3X: "); Serial.println(K3X,3);
  Serial.print("K1Y: "); Serial.print(K1Y);
  Serial.print(" K2Y: "); Serial.print(K2Y);
  Serial.print(" K3Y: "); Serial.println(K3Y,3);
}

