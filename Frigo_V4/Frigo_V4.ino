#include <math.h>
#include <string.h>


// Mofifications de la V4:
//  Action 1 : Changement de Tems en T° Ambiance                                          -> OK
//  Action 2 : Créer séquence de démarrage      
//         Si Interupteur enclenché = Sortie texte                                        -> OK
//         Sinon = Sortie LED uniquement                                                  -> Construire la fonction d'affichage LED (CF Action 3)
//        
//        Maximisation de la puissance de l'échangeur  (à cause du compresseur faible):
//        Temp Tc2 consigne = T° Eau de mer
// Action 3 :  Création de fonctions 
//              Affichage = Sortie print pour PC                                          -> OK
//              Gestion des leds                                                          -> A réaliser
//
// Action 4 : Modifie l'usage de l'interupteur (cela ne sert plus à forcer la pompe, mais à switcher entre 2 modes (ECO ou GLACONS))
//            Mode ECO = 
//                            Température de consigne Delta Temp interne frigo / exterieur (fonction de calcul)
//            Mode été = 

// défintion des pins d'entrée
  const int E_Amp=0,E_T1b=7,E_T1=1,E_T2=2,E_T3=3,E_T4=4,E_T5=5,E_T6=6,Inter_ext=8; 

// définition des pins de sortie
  const int Pompe_Broche=7,Comp_Frein_broche=6;
  const int LED1_Rouge_Broche=5,LED2_Orange_Broche=4,LED3_Jaune_Broche=3,LED4_Verte_Broche=2;

// définition des consignes du programme
  const int horloge=150; // défini le délai d'attente de la boucle lopp infinie.
  const int Nb_MMob=150,Niv_ON=1,Niv_OFF=0; 
  const int Tc2_Max=30,TEv2_C=4;
  const int Comp_Tmax=36,Comp_Tmax_demmarrage=30;
  const float Vref=5.0; //to be defined at 1.1V if option selected : analogReference(INTERNAL), else 5.0V 

// variables de calcul
  unsigned long temps;
  float UPWME1,Utemp1,Vtemp1,Vtemp1b,Utemp2,Vtemp2,Utemp3,Vtemp3,Utemp4,Vtemp4,Utemp5,Vtemp5,Utemp6,Vtemp6;
  float PElec,Tc,Tc2,TEv1,TEv2,TEm,TAmb;
  float Tc2_ecart,Tc2_ecart_nbpas=10,Tc2_ecart_pas=0.4;
  int Alarme1,Alarme2,Alarme3,Alarme4;

//Etat interupteur
  int Old_Inter=3,Inter;

// variables globales du circuit
  float  Circuit_Intensite;
  int Tc2_Consigne=25;
  int T_frigo_C=7,T_frigo_demmarage=9;

//variables du compresseur
  int Comp_Frein_Etat;

//variables de la pompe
  const int Pompe_Periode=10,Temps_Init_TEm=20000;
  int  Pompe_Etat=0,Pompe_Tps;
  float Pompe_VA,Pompe_Test_on_off=0.00,Pompe_Consigne=0;

//Variables d'affichage des LEDS
  const int LED_Periode=10;
  int LED_Tps;


// ajouter moyenne mobile de toutes les valeurs de température (en intégrant une boucle temporelle). -> différent de la mesure d'intensité.
// Pilotage compresseur en direct on/off depuis une sortie digitale

// Variables à modifier:
//  Pompe = Pompe_Broche (Pompe_Broche), Pompe_Etat (Pompe_Etat), Pompe_Tps (Pompe_Tps), Pompe_Periode(Pompe_Periode), Pompe_Test_on_off (Pompe_Test_on_off), Pompe_Consigne(Pompe_Consigne), Pompe_VA(Pompe_VA)
//  Compresseur = Comp_Broche (Comp_Broche), Comp_Frein_broche(Comp_Frein_broche), Comp_Frein_Etat(Comp_Frein_Etat),Comp_Limite_Temp
//  Consignes =
//              Tc2_Consigne (Tc2_Consigne), Tc2_ecart (Tc2_ecart), Tc2_ecart_nbpas=10 (Tc2_ecart_nbpas), Tc2_ecart_pas=0.4 (Tc2_ecart_pas)
//              T_frigo_C= 7°C
//              Tev2 = 10°C
//              TEv2_C = 4°C valeur optimale d retour d'évaporateur (doit être entre 0 et 4°C)      

void setup() { //fonction d'initialisation de la carte
   //contenu de l'initialisation

   // Priorité 1 = On bloque le compresseur
      pinMode(Comp_Frein_broche,OUTPUT);
      digitalWrite(Comp_Frein_broche,HIGH);
  
  // priorité 2: définition du voltage de référence (5V fonctionne, pas 1.1V)
   //analogReference(INTERNAL); //if option selected : analogReference(INTERNAL), Vref has to be defined at 1.1V, else 5.0V
   
   //Priorité 3:  définition des Pins de connexions de la board arduino
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

      pinMode(LED1_Rouge_Broche,OUTPUT);
      pinMode(LED2_Orange_Broche,OUTPUT);
      pinMode(LED3_Jaune_Broche,OUTPUT);
      pinMode(LED4_Verte_Broche,OUTPUT);

   // Ouverture de la connexion série
      Serial.begin(9600);

  //  Test des LEDS
      digitalWrite(LED1_Rouge_Broche,HIGH);
      delay(1000);
      digitalWrite(LED2_Orange_Broche,HIGH);
      delay(1000);
      digitalWrite(LED3_Jaune_Broche,HIGH);
      delay(1000);
      digitalWrite(LED4_Verte_Broche,HIGH);
      delay(1000);

      digitalWrite(LED1_Rouge_Broche,LOW);
      digitalWrite(LED2_Orange_Broche,LOW);
      digitalWrite(LED3_Jaune_Broche,LOW);
      digitalWrite(LED4_Verte_Broche,LOW);

   // initialisation des variables :
      Comp_Frein_Etat=Niv_ON;  // blocage du compresseur
      Alarme1=Niv_OFF;
      Alarme2=Niv_OFF;
      Alarme3=Niv_OFF;
      Alarme4=Niv_OFF;

   Serial.println("Séquence d'initialisation terminée, Démarrage !");

}

void loop() { //fonction principale, elle se répète (s'exécute) à l'infini
  Acquisition_capteurs(); // Procédure d'acquisition des capteurs de temp, intensité et assigne dans des variables globales. 
  Etat_des_Alarmes(); // Teste les conditions d'état d'alarme
  Force_Tem_SI_init_ou_inter_chg(Temps_Init_TEm); //Initialisation TEm + adapter les consignes si interupteur enclenché ou non
  Pilotage_Compresseur(); // Pilote le compresseur
  Pilotage_Pompe_eau(); // Procédure de contrôle de la pompe à eau
  Affichage_Led(); // Procédure de contrôle de l'affichage des LED
  Sortie_Serie_2(); // Envoi des données du frigo sur le port serie.
  delay(horloge); // Fait patienter la boucle avant la prochaine ittération
  }

// BLOC DE FONCTIONS ET PROCEDURES 

void Force_Tem_SI_init_ou_inter_chg(int Temps_en_ms){
  
// modification de la consigne du frigo si l'interupteur est change d'état (Mode Froid ++)
  // SI interupteur change d'état on réinitialise la Tem
  if(Old_Inter!=Inter) {
    
    digitalWrite(Pompe_Broche,HIGH);
    Pompe_Etat=1;
    Serial.println("Intitialisation TEm pendant " + String(Temps_en_ms) + " ms");
    delay(Temps_en_ms); // Fait patienter 15secondes pour initialisation de Teme
    digitalWrite(Pompe_Broche,LOW);
    Acquisition_capteurs();

    if (Inter==1) {
      T_frigo_C=2,
      T_frigo_demmarage=4,
      Tc2_Consigne=TEm+1;
      }
    else {
      T_frigo_C=7,
      T_frigo_demmarage=9,
      Tc2_Consigne=TEm+4;
      } // un calcul de l'isolation serait pertinent

    Old_Inter=Inter;
  }  
}


void Acquisition_capteurs(){
    
    // Recupération des valeurs des capteurs
  
    unsigned int x=0;
    float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0,AvNiv_T1b=0.0,AvNiv_T1=0.0,AvNiv_T2=0.0,AvNiv_T3=0.0,AvNiv_T4=0.0,AvNiv_T5=0.0,AvNiv_T6=0.0;
    
    for (int x = 0; x < Nb_MMob; x++){ //Get 150 samples of ACs Value
        AcsValue = analogRead(E_Amp);     //Read current sensor values   
         
        Samples = Samples + AcsValue;  //Add samples together
        delay (3); // let ADC settle before next sample 3ms
      }
    AvgAcs=Samples/Nb_MMob;//Taking Average of Samples

    // récupération des capteurs de température
    AvNiv_T1b=analogRead(E_T1b);
    AvNiv_T1=analogRead(E_T1);
    AvNiv_T2=analogRead(E_T2);
    AvNiv_T3=analogRead(E_T3);
    AvNiv_T4=analogRead(E_T4);
    AvNiv_T5=analogRead(E_T5);
    AvNiv_T6=analogRead(E_T6);
    Inter=digitalRead(Inter_ext);
    //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
    //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
    //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
    //you must change the offset according to the input voltage)
    //0.185v(185mV) is rise in output voltage when 1A current flows at input
    AcsValueF = (Vref/2 - (AvgAcs * (Vref / 1024.0)) )/0.185;

  // Calculs des variables du frigo et de la variable temps

   //UPWME1=(Niv_PWME1*5./1023); // Potentiometre
   Utemp1=(AvNiv_T1)*(Vref/1023*100);
   Vtemp1=AvNiv_T1;
   Vtemp1b=AvNiv_T1b;
   Utemp2=AvNiv_T2*(Vref/1023*100);
   Vtemp2=AvNiv_T2;
   Utemp3=AvNiv_T3*(Vref/1023*100);
   Vtemp3=AvNiv_T3;
   Utemp4=AvNiv_T4*(Vref/1023*100);
   Vtemp4=AvNiv_T4;
   Utemp5=AvNiv_T5*(Vref/1023*100);
   Vtemp5=AvNiv_T5;
   Utemp6=AvNiv_T6*(Vref/1023*100);
   Vtemp6=AvNiv_T6;
      
   Pompe_VA=AvgAcs * (Vref / 1024.0);
   Circuit_Intensite=abs((2.5 - (AvgAcs * (Vref / 1024.0)) )/0.185);
   PElec=Circuit_Intensite*12;
   
   temps=micros()/1000000;

  // assignation des valeurs de capteurs aux variables de calcul (faciliter la lecture)
   Tc=Utemp6;
   Tc2=Utemp3;
   TEv1=Utemp1;
   TEv2=Utemp2;
   TEm=Utemp4;
   TAmb=Utemp5;

  //reassignation des valeurs pour simulation à la maison
  //Tc=45;
  //Tc2=22;
  //TEv1=10;
  //TEv2=12;
  //TEm=22;
  //TAmb=25;  
  //Circuit_Intensite =1;
  //PElec=Circuit_Intensite*12;
  }

void Etat_des_Alarmes() { // Procédure de contrôle de l'affichage des LED
 // Configuration des voyants LED
  //   digitalWrite(LED1_Rouge_Broche,HIGH); // Allume LED Alarme 1 = Température compresseur
  //                         Alarme1=Niv_ON;
  //   digitalWrite(LED2_Orange_Broche,HIGH); // Allume LED Alarme 2 = Température Condenseur
  //                           Alarme2=Niv_ON;
  //   digitalWrite(LED3_Verte_Broche,HIGH);  // ALlume LED Alarme 3 = Température Frigo OK
  //                                 Alarme3=Niv_ON;
  //   digitalWrite(LED4_Jaune_Broche,HIGH);   // Eteint LED Alarme 4 = Compresseur sous tension
  //                                 Alarme4=Niv_ON;
  //Initialisation avant test :   
    //Alarme1=0; Mise en commentaire pour bloquer le compresseur tant que l'utilisateur ne réinitialise pas le frigo
    Alarme2=0;
    Alarme3=0;
    Alarme4=0;

  // TEST ALARME 1
    if (Alarme1=0) {
      if (Tc>Comp_Tmax) { if (Alarme1<=9) {Alarme1=9;}} // Configure l'alarme au niveau maximum si TC > TcMax
      if (Circuit_Intensite>3) {if (Alarme1<=7) {Alarme1=7;}} // Configure l'alarme1 au niveau 7 si l'intensité consommée > 3A
      if (Tc2>(Tc2_Max)) {if (Alarme1<=5) {Alarme1=5;}} // Configure l'alarme1 au niveau 5 si Tc2 > Tc2max = 30°C
      
      if (Tc<3 or Tc>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 3 - 45 °C
      if (Tc2<3 or Tc2>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 3 - 45 °C
      if (TEv1<0 or TEv1>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 0 - 45 °C
      if (TEv2<0 or TEv2>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 0 - 45 °C
      if (TEm<3 or TEm>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 3 - 45 °C
      if (TAmb<3 or TAmb>45) {if (Alarme1<=4) {Alarme1=4;}} // Configure l'alarme1 au niveau 4 si un capteur de température sort de la plage 3 - 45 °C
    }
  // TEST ALARME 2
      if (Tc2>(Tc2_Consigne+2)) {if (Alarme2<=6) {Alarme2=6;}} // Configure l'alarme2 au niveau 6 si Tc2 supérieure à 2°C de la consigne
      if (TEv2>TEv2_C) {if (Alarme2<=4) {Alarme2=4;}} // Configure l'alarme2 au niveau 2 si TEv2 supérieure à 4°C 
  
  // TEST ALARME 3
      if (Comp_Frein_Etat=Niv_OFF) {if (Alarme3<=9) {Alarme3=9;}} // Configure l'alarme3 au niveau 4 si le compresseur est sous tension
  // TEST ALARME 4  
      if (TEv1<T_frigo_C+6) {if (Alarme4<=2) {Alarme4=2;}} // Configure l'alarme4 au niveau 2 si TEv1 > 6degrés vs consigne frigo
      if (TEv1<T_frigo_C+4) {if (Alarme4<=4) {Alarme4=4;}} // Configure l'alarme4 au niveau 4 si TEv1 > 4degrés vs consigne frigo
      if (TEv1<T_frigo_C+3) {if (Alarme4<=6) {Alarme4=6;}} // Configure l'alarme4 au niveau 2 si TEv1 > 3degrés vs consigne frigo
      if (TEv1<T_frigo_C+2) {if (Alarme4<=8) {Alarme4=8;}} // Configure l'alarme4 au niveau 2 si TEv1 > 2degrés vs consigne frigo
      if (TEv1<=T_frigo_C+1) {if (Alarme4<=9) {Alarme4=9;}} // Configure l'alarme4 au niveau 2 si TEv1 > 1degrés vs consigne frigo
  //reassignation des valeurs pour simulation à la maison
  //Alarme1=10;
  //Alarme2=6;
  //Alarme3=4;
  //Alarme4=2;
  }

void Affichage_Led(){

  LED_Tps=floor(temps);
  LED_Tps=LED_Tps % LED_Periode;
  if((float(LED_Tps)/LED_Periode-float(Alarme1)/10)<0) {digitalWrite(LED1_Rouge_Broche,HIGH);} else {digitalWrite(LED1_Rouge_Broche,LOW);}
  if((float(LED_Tps)/LED_Periode-float(Alarme2)/10)<0) {digitalWrite(LED2_Orange_Broche,HIGH);} else {digitalWrite(LED2_Orange_Broche,LOW);}
  if((float(LED_Tps)/LED_Periode-float(Alarme3)/10)<0) {digitalWrite(LED3_Jaune_Broche,HIGH);} else {digitalWrite(LED3_Jaune_Broche,LOW);}
  if((float(LED_Tps)/LED_Periode-float(Alarme4)/10)<0) {digitalWrite(LED4_Verte_Broche,HIGH);} else {digitalWrite(LED4_Verte_Broche,LOW);}
}


void Pilotage_Compresseur() {
  // algortithme de pilotage du compresseur
  // Stopper Moteur aux conditions suivantes  :
  //Condition 1 = Température compresseur > Tmax -> Mise en sécurité compresseur
  //Condition 2 => Si Tc2 > Tc2_Max -> PB de condenseur (pompe)
  //Condition 3 => Si T enceinte < T consigne du frigo 
  //Information 4 => Compresseur sous tension
  //Information 5 => Si Tev2 a une température anormale > 15°C (après lancement) -> PB circuit frigo : manque de gaz
  // Relancer moteur aux conditions suivantes (= opposé des 4 conditions au dessus).

  // VÉRIFICATION N°1 = Témpérature du compresseur
  if (Alarme1!=0) {
    Comp_Frein_Etat=Niv_ON;
    digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
  }
  else {
    // VÉRIFICATION N°2 = Refroidissement Condenseur : Témpérature de compression vérification 
    if (Tc2>Tc2_Max){
      Comp_Frein_Etat=Niv_ON;
      digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
    }
    else {
      // VÉRIFICATION N°3 = Si TConsigne du frigo atteinte
      if (TEv1<T_frigo_C){
        Comp_Frein_Etat=Niv_ON;
        digitalWrite(Comp_Frein_broche,HIGH); // Bloque le compresseur
        }
      else {
        // VÉRIFICATION N°4 = Si T du compresseur est valide pour un redémarrage et si T frigo > Valeur supérieure à consigne + hystérésis
        if (TEv1>=T_frigo_demmarage) {
          if(Tc<Comp_Tmax_demmarrage) {
            Comp_Frein_Etat=Niv_OFF;
            digitalWrite(Comp_Frein_broche,LOW); // Débloque le compresseur.
            }
          }                        
        }
      }
    }
}

void Pilotage_Pompe_eau() {
  // algorithme de pilotage pompe à eau
  // Calcul de l'écart de Tc2 à la consigne de fonctionnement (Si Inter enclenché, on force la pompe à 100%)
  if (Inter==1) {
    Tc2_ecart=Inter*100;
    }
  else {
    Tc2_ecart=Tc2-Tc2_Consigne;
    }
  
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
  }

void Sortie_Serie_2() 
  {
  // Transmission des données de suivi sur PC
  //Serial.print("T=");
  String Text= String("T=" + String(temps) + "s|Inter=" + String(Inter) +"|I=" + String(Circuit_Intensite) +"A|P=" + String(PElec) + "W|AL1234=" + String(Alarme1) 
              + String(Alarme2) + String(Alarme3) + String(Alarme4) + "|Comp:Frein=" + String(Comp_Frein_Etat) + "|Tpompe=" + String(Pompe_Tps) +"|s %=" 
              + String(Pompe_Consigne) + "|Temp(°C):Tc=" + String(Tc) + "|Tc2=" + String(Tc2)+ "|Tc2_C=" + String(Tc2_Consigne) + "|Tc2_Max=" + String(Tc2_Max)+ "|Tev1=" + String(TEv1)+ "|Tev1_C=" + String(T_frigo_C) 
              + "|Tev2=" + String(TEv2) + "|Tev2_C=" + String(TEv2_C) 
              + "|TEm=" + String(TEm) + "|TAmb=" + String(TAmb));
  Serial.println(Text);
  }
