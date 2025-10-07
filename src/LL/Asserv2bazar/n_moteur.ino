


float getMeanSpeed(float dt){
  int length = sizeof(mesures)/sizeof(mesures[0]);
  //ajout d'une nouvelle mesure et suppression de l'ancienne
  for (int i=length-1;i>0;i--){
      mesures[i]=mesures[i-1];
  }
  mesures[0] = getSpeed(dt);

  //Calcul d'une moyenne pour lisser les mesures qui sont trop dipersées sinon
  float sum=0;
  for (int i=0;i<length;i++){
    sum+=mesures[i];
  }

  //affichage debug
  #if 0
  for(int i=0;i<length;i++){
    Serial.print(mesures[i]);
    Serial.print(" , ");
  }
  Serial.println(sum/length);
  #endif

  return sum/length;
}


float getSpeed(float dt){  
  int N = count - vieuxCount; //nombre de fronts montant et descendands après chaque loop
  float V = ((float)N/(float)nb_trous)*distanceUnTour/(dt*1e-3); //16 increments -> 1 tour de la roue et 1 tour de roue = 79 mm 
  vieuxCount=count;
  vieuxTemps=millis();
  return V;
}


float PID(float cons, float mes, float dt) {
  // Adjust the measured speed based on the sign of the desired speed
  float adjustedMes = (cons < 0) ? -mes : mes;

  // Calculate the error
  float e = cons - adjustedMes;

  // Proportional term
  float P = Kp * e;

  // Integral term
  integral = integral + e * dt;
  float I = Ki * integral;

  #if 0
  // Derivative term
  derivee = (e - vieuxEcart) / dt;
  vieuxEcart = e;
  float D = Kd * derivee;
  #endif

  return P + I;
}