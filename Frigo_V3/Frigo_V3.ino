#include <math.h>

// défintion des pins d'entrée
    
  const int E_Amp=0,E_T1b=7,E_T1=1,E_T2=2,E_T3=3,E_T4=4,E_T5=5,E_T6=6,Inter_ext=8; 

// définition des pins de sortie
  const int Pompe_Broche=7,Comp_Frein_broche=6;
  const int Alarme1_Broche=5,Alarme2_Broche=4,Alarme3_Broche=3,Alarme4_Broche=2;

// définition des consignes du programme
  const int Nb_MMob=150,Niv_ON=10,Niv_OFF=0; 
  const int Tc2_Consigne=30,Tc2_Max=34,T_frigo=2,T_frigo_demmarage=6;
  const int Comp_Tmax=50,Comp_Tmax_demmarrage=40;

// variables de calcul
  unsigned long temps;
  float UPWME1,Utemp1,Vtemp1,Vtemp1b,Utemp2,Vtemp2,Utemp3,Vtemp3,Utemp4,Vtemp4,Utemp5,Vtemp5,Utemp6,Vtemp6;
  float PElec,Tc,Tc2,TEv1,TEv2,TEme,TEms;

  float Tc2_ecart,Tc2_ecart_nbpas=10,Tc2_ecart_pas=0.4;
  int Alarme1,Alarme2,Alarme3,Alarme4;
//Etat interupteur
  int Inter;

//variables du compresseur
  int Comp_Frein_Etat;

//variables de la pompe
  const int Pompe_Periode=10;
  int  Pompe_Etat=0,Pompe_Tps;
  float Pompe_Intensite,Pompe_VA,Pompe_Test_on_off=0.00,Pompe_Consigne=0;


// ajouter moyenne mobile de toutes les valeurs de température (en intégrant une boucle temporelle). -> différent de la mesure d'intensité.
// Pilotage compresseur en direct on/off depuis une sortie digitale

// Variables à modifier:
//  Pompe = Pompe_Broche (Pompe_Broche), Pompe_Etat (Pompe_Etat), Pompe_Tps (Pompe_Tps), Pompe_Periode(Pompe_Periode), Pompe_Test_on_off (Pompe_Test_on_off), Pompe_Consigne(Pompe_Consigne), Pompe_Intensite (Pompe_Intensite), Pompe_VA(Pompe_VA)
//  Compresseur = Comp_Broche (Comp_Broche), Comp_Frein_broche(Comp_Frein_broche), Comp_Frein_Etat(Comp_Frein_Etat),Comp_Limite_Temp
//  Consignes =
//              Tc2_Consigne (Tc2_Consigne), Tc2_ecart (Tc2_ecart), Tc2_ecart_nbpas=10 (Tc2_ecart_nbpas), Tc2_ecart_pas=0.4 (Tc2_ecart_pas)
//              T_frigo= 4°C
//              Tev2 = 10°C      


void setup() { //fonction d'initialisation de la carte
   //contenu de l'initialisation
   //pinMode(E0, INPUT);

   analogReference(INTERNAL); 
   
   pinMode(E_Amp, INPUT);
   pinMode(E_T1b, INPUT);
   pinMode(E_T1,INPUT);
   pinMode(E_T2,INPUT);
   pinMode(E_T3,INPUT);
   pinMode(E_T4,INPUT);
   pinMode(E_T5,INPUT);
   pinMode(E_T6,INPUT);
   pinMode(Inter_ext,INPUT);
   pinMode(Pompe_Broche,OUTPUT);
   pinMode(Comp_Frein_broche,OUTPUT);

   pinMode(Alarme1_Broche,OUTPUT);
   pinMode(Alarme2_Broche,OUTPUT);
   pinMode(Alarme3_Broche,OUTPUT);
   pinMode(Alarme4_Broche,OUTPUT);

   Serial.begin(9600);

   // initialisation des variables :
      Comp_Frein_Etat=Niv_ON;  // blocage du compresseur
      Alarme1=Niv_OFF;
      Alarme2=Niv_OFF;
      Alarme3=Niv_OFF;
      Alarme4=Niv_OFF;

   Serial.println("Démarrage !");
}
void loop() { //fonction principale, elle se répète (s'exécute) à l'infini
   //contenu du programme
   //niveau = analogRead(E0) ; // Lecture du signal sur l'entrée E1
   //uPWM = niveau * 5. / 1023. ;
   //sortie_pwm=int(niveau*255./1023);
  

   // partie spécifique capteur d'intensité ACS712
   
    unsigned int x=0;
    float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0,AvNiv_T1b=0.0,AvNiv_T1=0.0,AvNiv_T2=0.0,AvNiv_T3=0.0,AvNiv_T4=0.0,AvNiv_T5=0.0,AvNiv_T6=0.0;
    float Niv_T1b=0.0,Niv_T1=0.0,Niv_T2=0.0,Niv_T3=0.0,Niv_T4=0.0,Niv_T5=0.0,Niv_T6=0.0;
    float Sample_T1b=0.0,Sample_T1=0.0,Sample_T2=0.0,Sample_T3=0.0,Sample_T4=0.0,Sample_T5=0.0,Sample_T6=0.0;
    
      for (int x = 0; x < Nb_MMob; x++){ //Get 150 samples
      AcsValue = analogRead(E_Amp);     //Read current sensor values   
            
    // récupération des capteurs
       Niv_T1b=analogRead(E_T1b);
       Niv_T1=analogRead(E_T1);
       Niv_T2=analogRead(E_T2);
       Niv_T3=analogRead(E_T3);
       Niv_T4=analogRead(E_T4);
       Niv_T5=analogRead(E_T5);
       Niv_T6=analogRead(E_T6);
       Inter=digitalRead(Inter_ext);

      Samples = Samples + AcsValue;  //Add samples together
      Sample_T1b= Sample_T1b= + Niv_T1b;
      Sample_T1= Sample_T1= + Niv_T1;
      Sample_T2= Sample_T2= + Niv_T2;
      Sample_T3= Sample_T3= + Niv_T3;
      Sample_T4= Sample_T4= + Niv_T4;
      Sample_T5= Sample_T5= + Niv_T5;
      Sample_T6= Sample_T6= + Niv_T6;
      
      delay (3); // let ADC settle before next sample 3ms
    }
    AvgAcs=Samples/Nb_MMob;//Taking Average of Samples
    //AvNiv_T1b=Sample_T1b/Nb_MMob;
    //AvNiv_T1=Sample_T1/Nb_MMob;
    //AvNiv_T2=Sample_T2/Nb_MMob;
    //AvNiv_T3=Sample_T3/Nb_MMob;
    //AvNiv_T4=Sample_T4/Nb_MMob;
    //AvNiv_T5=Sample_T5/Nb_MMob;
    //AvNiv_T6=Sample_T6/Nb_MMob;

    AvNiv_T1b=Niv_T1b;
    AvNiv_T1=Niv_T1;
    AvNiv_T2=Niv_T2;
    AvNiv_T3=Niv_T3;
    AvNiv_T4=Niv_T4;
    AvNiv_T5=Niv_T5;
    AvNiv_T6=Niv_T6;

    //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
    //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
    //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
    //you must change the offset according to the input voltage)
    //0.185v(185mV) is rise in output voltage when 1A current flows at input
    
    AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185;

// Calculs des tensions
   
   //UPWME1=(Niv_PWME1*5./1023); // Potentiometre
   
   Utemp1=(AvNiv_T1)*(1.1/1023*100);
   Vtemp1=AvNiv_T1;
   Vtemp1b=AvNiv_T1b;
   Utemp2=AvNiv_T2*(1.1/1023*100);
   Vtemp2=AvNiv_T2;
   Utemp3=AvNiv_T3*(1.1/1023*100);
   Vtemp3=AvNiv_T3;
   Utemp4=AvNiv_T4*(1.1/1023*100);
   Vtemp4=AvNiv_T4;
   Utemp5=AvNiv_T5*(1.1/1023*100);
   Vtemp5=AvNiv_T5;
   Utemp6=AvNiv_T6*(1.1/1023*100);
   Vtemp6=AvNiv_T6;
      
   Pompe_VA=AvgAcs * (1.1 / 1024.0);
   Pompe_Intensite=abs((2.5 - (AvgAcs * (1.1 / 1024.0)) )/0.185);
   PElec=Pompe_Intensite*12;
   
   temps=micros()/1000000;


// assignation des valeurs de capteurs aux variables de calcul (faciliter la lecture)
  //
  // Tc=Utemp6;
  // Tc2=Utemp4;
  // TEv1=Utemp1;
  // TEv2=Utemp2;
  // TEme=Utemp5;
  // TEms=Utemp3;
  //

   Tc=Utemp6;
   Tc2=Utemp3;
   TEv1=Utemp1;
   TEv2=Utemp2;
   TEme=Utemp4;
   TEms=Utemp5;




// algortithme de pilotage du compresseur

      // Stopper Moteur aux conditions suivantes  :
            //Condition 1 = Température compresseur > Tmax
                  // -> Mise en sécurité compresseur
            //Condition 2 => Si Tc2 > Tc2_Max
                  // -> PB de condenseur (pompe)
            //Condition 3 => Si T enceinte < T consigne du frigo
            //Information 4 => Compresseur sous tension
            //Information 5 => Si Tev2 a une température anormale > 15°C (après lancement) 
                  // -> PB circuit frigo : manque de gaz
            

    // Relancer moteur aux conditions suivantes (= opposé des 4 conditions au dessus).

      // VÉRIFICATION N°1 = Témpérature du compresseur
      if (Tc>Comp_Tmax) {
                            Comp_Frein_Etat=Niv_ON;
                            digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
                            digitalWrite(Alarme1_Broche,HIGH); // Allume LED Alarme 1 = Température compresseur
                            Alarme1=Niv_ON;
                            digitalWrite(Alarme4_Broche,LOW);  // Eteint LED Alarme 4 = Compresseur sous tension
                            Alarme4=Niv_OFF;
                         }
      else {
            digitalWrite(Alarme1_Broche,LOW); // Eteint LED Alarme 1 = Température compresseur
            Alarme1=Niv_OFF;
            
            // VÉRIFICATION N°2 = Refroidissement Condenseur : Témpérature de compression vérification 
            if (Tc2>Tc2_Max){
                              Comp_Frein_Etat=Niv_ON;
                              digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
                              digitalWrite(Alarme2_Broche,HIGH); // Allume LED Alarme 2 = Température Condenseur
                              Alarme2=Niv_ON;
                              digitalWrite(Alarme4_Broche,LOW); // Eteint LED Alarme 4 = Compresseur sous tension
                              Alarme4=Niv_OFF;
                            }
            else {
                  digitalWrite(Alarme2_Broche,LOW); // Eteint LED Alarme 2 = Température Condenseur
                  Alarme2=Niv_OFF;

                  // VÉRIFICATION N°3 = Si TConsigne du frigo atteinte
                  if (TEv1<T_frigo){
                                    Comp_Frein_Etat=Niv_ON;
                                    digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
                                    digitalWrite(Alarme3_Broche,HIGH);  // ALlume LED Alarme 3 = Température Frigo OK
                                    Alarme3=Niv_ON;
                                    digitalWrite(Alarme4_Broche,LOW);   // Eteint LED Alarme 4 = Compresseur sous tension
                                    Alarme4=Niv_OFF;
                                    }
                  else {
                         
                         
                        // VÉRIFICATION N°4 = Si T du compresseur est valide pour un redémarrage et si T frigo > Valeur supérieure à consigne + hystérésis
                        
                         if (TEv1>T_frigo_demmarage) {
                                                       digitalWrite(Alarme3_Broche,LOW);  // Eteint LED Alarme 3 = Température Frigo OK
                                                       Alarme3=Niv_OFF;
                                                       if(Tc<Comp_Tmax_demmarrage) {
                                                                                    Comp_Frein_Etat=Niv_OFF;
                                                                                    //digitalWrite(Comp_Frein_broche,LOW); // Débloque le compresseur.
                                                                                    digitalWrite(Alarme4_Broche,HIGH); // Allume LED Alarme 4 = Compresseur sous tension
                                                                                    Alarme4=Niv_ON;
                                                                                    }
                                                     }
                                                     
                        }
                }
          }

   
      
// algorithme de pilotage pompe à eau

  //pilotage pompe à eau

      // Calcul de l'écart de Tc2 à la consigne de fonctionnement
        Tc2_ecart=Tc2-Tc2_Consigne;
        Tc2_ecart=Inter;
      // Calcul de la consigne de % de la pompe
      
      if (Tc2_ecart>0) {
                        Pompe_Consigne= min(floor(Tc2_ecart*Tc2_ecart_nbpas/Tc2_ecart_pas),100);
                        }
      else {
            Pompe_Consigne=0;
      }

      // Calcul temporel (modulo du temps passé sur la période de fonctionnement de la pompe définie en consigne générale)
        Pompe_Tps=floor(temps);
        Pompe_Tps=Pompe_Tps % Pompe_Periode;
      
      // Définition si la pompe doit être allumée ou non ( La pompe est sous tension X % (Pompe_Consigne) du temps de la période (Pompe_Periode))
       
           Pompe_Test_on_off=float(Pompe_Tps)/Pompe_Periode-Pompe_Consigne/100;

           if (Pompe_Test_on_off<0) {
                                    digitalWrite(Pompe_Broche,HIGH);
                                    Pompe_Etat=1;
                                    }
           else {
                  digitalWrite(Pompe_Broche,LOW);
                  Pompe_Etat=0;
                 }

     // tempo=abs(cos(temps*2*3.14/120));
     // Pompe_Consigne=floor(255*tempo); 
      //Pompe_Consigne=0;

      // pilotage 100% puissance PWM (-> Digital simple + temporisation 0 -> 100% par pas de 10%)
      //analogWrite(Pompe_Broche,floor(Pompe_Consigne)); // inversion commande pompe (Transistor PNP)
      
 // Configuration des voyants LED

   //   digitalWrite(Alarme1_Broche,HIGH); // Allume LED Alarme 1 = Température compresseur
   //                         Alarme1=Niv_ON;
   //   digitalWrite(Alarme2_Broche,HIGH); // Allume LED Alarme 2 = Température Condenseur
   //                           Alarme2=Niv_ON;
   //   digitalWrite(Alarme3_Broche,HIGH);  // ALlume LED Alarme 3 = Température Frigo OK
   //                                 Alarme3=Niv_ON;
   //   digitalWrite(Alarme4_Broche,HIGH);   // Eteint LED Alarme 4 = Compresseur sous tension
   //                                 Alarme4=Niv_ON;

// Transmission des données de suivi sur PC
  
  Serial.print("T=;");
  Serial.print(temps);
  Serial.print(";s");

  Serial.print("T_pompe=;");
  Serial.print(Pompe_Tps);
  Serial.print(";s;E:");
  Serial.print(Pompe_Etat);
  Serial.print(";");

   

  //Serial.print("Vtemp6= ");
  //Serial.print(Vtemp6);
  Serial.print(";Tc=;");
  Serial.print(Tc);
  //Serial.print(Utemp6);
  //Serial.print(";°C ;");

  //Serial.print("Vtemp4= ");
  //Serial.print(Vtemp4);
  Serial.print(";Tc2=;");
  Serial.print(Tc2);
  //Serial.print(Utemp4);
  //Serial.print(";°C ;");

  //Serial.print("Vtemp1= ");
  //Serial.print(Vtemp1);
  Serial.print(";Tev1=;");
  Serial.print(TEv1);
  //Serial.print(Utemp1);
  //Serial.print(";°C ;");
 
  //Serial.print("Vtemp2= ");
  //Serial.print(Vtemp2);
  Serial.print(";Tev2=;");
  Serial.print(TEv2);
  //Serial.print(Utemp2);
  //Serial.print(";°C ;");

  //Serial.print("Vtemp5= ");
  //Serial.print(Vtemp5);
  Serial.print(";TEme=;");
  Serial.print(TEme);
  //Serial.print(Utemp5);
  //Serial.print(";°C ;");
  
  //Serial.print("Vtemp3= ");
  //Serial.print(Vtemp3);
  Serial.print(";TEms=;");
  Serial.print(TEms);
  //Serial.print(Utemp3);
  //Serial.print(";°C ;");

  //Serial.print("U_PWM=;");
  //Serial.print(UPWME1);

  Serial.print(";EFC=;");
  Serial.print(Comp_Frein_Etat);

  Serial.print(";EP=;");
  Serial.print(Pompe_Etat);
  Serial.print(";PCT_P=;");
  Serial.print(Pompe_Consigne);  
  Serial.print("%;");
  Serial.print("A=;");
  Serial.print(Pompe_Intensite);

  Serial.print(";PElec=;");
  Serial.print(PElec);
  Serial.print("W;");

  Serial.print(";AL1=;");
  Serial.print(Alarme1);
  Serial.print(";AL2=;");
  Serial.print(Alarme2);
  Serial.print(";AL3=;");
  Serial.print(Alarme3);
  Serial.print(";AL4=;");
  Serial.print(Alarme4);
  Serial.print("Inter=");
  Serial.print(Inter);
  Serial.println(" ");

 // Serial.print("Tension mesurée= ");
 // Serial.print(uPWM);
 // Serial.print(" V. ");
 // Serial.print("PWM= ");
 // Serial.print(sortie_pwm);
 // Serial.print(". Utemp= ");
 // Serial.print(Utemp);
 // Serial.print("°C. Uamp= ");
 // Serial.print(Uamp);
 // Serial.println(" A");
  delay(150);
}
