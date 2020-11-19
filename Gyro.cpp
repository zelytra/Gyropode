#include "mbed.h"
#include "MPU6050.h"
#include "rtos.h"
#include "math.h"
#include "FastPWM.h"

#include <cstdlib>

#define Te_ms 5
//#define To 0.5
#define Te Te_ms/1000.0
#define EcCons 0.1


//------------------------//'
//       Projet E&R       //
//        Gyropode        //
//------------------------//
//     Alexandre Petit    //
//     Micheal Laporte    //
//        S4 2020         //
//------------------------//
//  Programmer pour carte //
//   MBED NUCKLEO F303K8  //
//------------------------//



//Définition des E/S//
DigitalOut  myled(LED3);            //Sortie TOR pour LED du µC
AnalogIn    NivBatt(A1);            //Entrée analogique pour niveau de tension de batterie

RawSerial   pc(USBTX, USBRX);       //Tx, Rx liaison série vers USB pour analyse Terminal
RawSerial   HC06(PA_9,PA_10);       //Tx, Rx Bluetouth
MPU6050     mpu(I2C_SDA, I2C_SCL);  //Bus I2C pour le gyroscope MPU6050

FastPWM      M11(D9);                //Sortie PWM M1.1
FastPWM      M12(D10);               //Sortie PWM M1.2
FastPWM      M21(D11);               //Sortie PWM M2.1
FastPWM      M22(D12);               //Sortie PWM M2.2
//------------------//


//Variable global//
double ax, ay, az;          //Valeurs des axes en m/s²
double gx, gy, gz;          //Valeurs accéleration des axes rad/s
float dataG[3];             //Stockages des valeurs dans des tableaux 0=x, 1=y, 2=z | G pour Gravité soit Accéleration
float dataA[3];             //Stockages des valeurs dans des tableaux 0=x, 1=y, 2=z | A pour vitesse Angulaire soit une position

double Angle;               //Valeur de l'arctangante de Gx par Gy
double AngleTo;             //Valeur de l'intégration de la vitesse et multiplication avec To
double AngleNF;            //Valeur de l'addition des Teta sans application du filtre
double AngleF;             //Valeur de l'angle après filtrage
double A,B,AEc,BEc;                 //Coéficients du filtre
double AngleFP;                  //Echantillonage précédent
double Rc,EcFiltre,EcFiltreP,ErreurEcP,ErreurEc;
int i;

double To=1;

char chaineAtraiter[50];
volatile int flagChaine = 0;

char msg[100];                                       //Tableau pour stockage du message BT

double erreur;                                       //Variable pour calcule de l'erreur
double AngleOffset=-0.0299;                                  //Commande voulue pour stabilisation du gyropode
double Kp=2.93,Kd=0.1234,KpEc=0.2732,KdEc=0.0375;           //Coéficient pour asservissement
double AngleCons=0,DeltaAlpha=0,DeltaAlphaOffset;
double ToEc=0.1;


//---------------//



void Control(void) //Fonction de calcule de l'angle et d'asservissement
{   
    //Acquisition des datas//
    mpu.getAccelero(dataA);
    mpu.getGyro(dataG);
    ay=dataA[1];
    az=dataA[2];
    gx=dataG[0];
    //---------------------//

    //Calcul de l'angle//
    Angle=  atan2(ay,az);               //Calcule de l'angle en régime permanant (basse fréquence)
    AngleTo =  gx*To;                   //Calcule de l'angle en régime transitoir (haute fréquence)
    AngleNF = Angle+AngleTo;            //Addition des calcule d'angle celon le régime
    AngleF  = (A*AngleNF)+(B*AngleF);   //Application du filtre complémentaire
    AngleFP= AngleF;
    //------------------//

    //Calcule de l'Erreur//
    erreur=AngleCons+AngleOffset-AngleF;
    Rc =(Kp*erreur)-(Kd*gx);
    //-------------------//

    //Saturation de Rc//
    if (Rc<-0.35)  Rc=-0.35;
    if (Rc>0.35)   Rc=0.35;
    //----------------//

    //Filtre Ec pour l'erreur de position//
    EcFiltre=AEc*Rc+BEc*EcFiltreP;
    EcFiltreP=EcFiltre;
    //---------//

    //Calcule de Ec//
    ErreurEc = EcCons-EcFiltre;
    AngleCons=-(KpEc*(ErreurEc)+KdEc*(ErreurEc-ErreurEcP));
    ErreurEcP=ErreurEc;
    //-------------//

    //Saturation Angle Consigne//
    if (AngleCons<-0.5)  AngleCons=-0.5;
    if (AngleCons>0.5)   AngleCons=0.5;
    //-------------------------//

    //Saturation EC+Offset//
    if(Rc>0)  DeltaAlphaOffset=Rc+0.15;
    if(Rc<0)  DeltaAlphaOffset=Rc-0.15;
    //--------------------//

    //Control Moteur//
    M11.write(0.5+DeltaAlphaOffset);
    M12.write(0.5-DeltaAlphaOffset);
    M21.write(0.5-DeltaAlphaOffset);
    M22.write(0.5+DeltaAlphaOffset);
    //--------------//
}
//Création de l'intérruption pour échantillonage à période définie
RtosTimer control(mbed::callback(Control),osTimerPeriodic);

void Niv_Batt(void)
{
    //Si Niv Batterie inférieur à 11.8V alors allumer LED
    if((NivBatt.read())<(float)0.76) {
        myled=1;
    }
    if((NivBatt.read())>=(float)0.76) {
        myled=0;
    }
}

//-----------------------//
void angle_init(void)
{
    pc.printf("Mise a 0 en cour...\r\n");
    for(i=0; i<500; i++) {
        AngleOffset+=AngleF;
        pc.printf("%3.0f%%\r\n",((i*100.0)/500.0));
    }
    AngleOffset/=500.0;
    pc.printf("Mise a 0 TERMINE\r\n");
}

void reception()
{
    static char chaine[50];
    static int i=0;
    char ch = pc.getc();
    if ((ch==13) || (ch==10)) {
        chaine[i]='\0';
        strcpy(chaineAtraiter, chaine);
        flagChaine = 1;
        i=0;
    } else {
        chaine[i]=ch;
        i++;
    }
}

void traitement()
{
    char commande[6];
    char valeur[20];

    strcpy(commande, strtok(chaineAtraiter, " "));
    strcpy(valeur,  strtok(NULL, " "));

    if (strcmp(commande,"To")==0) {
        To=atof(valeur);
    } else if (strcmp(commande,"KpEc")==0) {
        KpEc=atof(valeur);
    } else if (strcmp(commande,"KdEc")==0) {
        KdEc=atof(valeur);
    } else if (strcmp(commande,"Kd")==0) {
        Kd=atof(valeur);
    } else if (strcmp(commande,"Kp")==0) {
        Kp=atof(valeur);
    }
}

void Initialisation(void)
{
    //BT//
    int i=0;
    HC06.baud(9600);
    pc.baud(9600);
    //--//

    //Coef//
    A=Te/(Te+To);
    B=To/(Te+To);

    AEc = 1/(1+ToEc/Te);
    BEc = ToEc/Te/(1+ToEc/Te);
    //----//


    //Set Range du capteur//
    mpu.setGyroRange (MPU6050_GYRO_RANGE_2000);
    mpu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    //--------------------//

    //Test connections MPU//
    pc.printf("MPU6050 initialize \n\r");
    pc.printf("MPU6050 testConnection \n\r");
    bool mpu6050TestResult = mpu.testConnection();
    if(mpu6050TestResult) {
        pc.printf("MPU6050 test passed \n\r");
    } else {
        pc.printf("MPU6050 test failed \n\r");
        while(1);
    }
    //--------------------//


    control.start(Te_ms);      // Lancement du Timer pour échantillonage
    //angle_init();              //Appelle de la fonction de mise a zero

    M11.period_us(50);
    M12.period_us(50);
    M21.period_us(50);
    M22.period_us(50);


    pc.attach(&reception);

//--------------//
}

int main()
{


    Initialisation();


    while(1) {

        Niv_Batt();         //Analyse du niveau de batterie
        pc.printf("%lf %lf %lf %lf\n\r",Angle,AngleF,erreur,ErreurEc);

        //Communication Bluetooth//
        if(HC06.readable()) {
            msg[i]=HC06.getc();
            pc.printf("%c\r\n",msg[i]);
            /*
            if (msg[i]=='!') {
                sscanf(msg,"%lf!",&Kprim);
                pc.printf("Kd:%f",Kd);
                i++;
                msg[i]='\0';
                pc.printf("\r\n\r\n");
                //motor_write(sens1,v1,sens2,v2);
                i=0;
            } else {
                i++;
            }
            */
            //-----------------------//

            pc.printf("%c",msg);
        }
        

        //pc.printf("%f %f %f %f %f %f\n\r",erreur,Rc,EcFiltre,ToEc,AEc,BEc);
//pc.printf("%f %f\n",Kp,Kd);
        if (flagChaine) {
            traitement();
            flagChaine = 0;
        }
    }
}
