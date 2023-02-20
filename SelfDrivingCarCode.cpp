#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

RaspiCam_Cv Camera;

Mat cadru, matriceTransformarePerpectiva, cadruVedereDeDeasupra, cadruGri, cadruPrag, cadruMargine, cadruFinal, cadruFinalCopie, cadruFinalCopie1;
Mat ROIBanda, ROIFinalBanda;
int pozBandaStg, pozBandaDrpt, centruBanda, Rezultat, finalBanda,frameCenter=185;

Point2f sursa[] = {Point2f(70,135),Point2f(330,135),Point2f(30,185), Point2f(370,185)};//maginile cadrului sursa
Point2f destinatie[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};//marginile cadrului a carui perspectica a fost modficat

vector<int> histogramaBanda;
vector<int> histogramaFinalBanda;

stringstream ss;//pentru titlurile imaginilor preluate de la camera si prelucrate

//daca DEBUG are valoarea 1 se pot vedea imagini preluate de la camera
//daca are valoarea 1 are viteza maxima
#define DEBUG 0	

void seteazaParamCamera ( int argc,char **argv, RaspiCam_Cv &Camera )
{
	Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
	Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
	Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,40 ) );
	Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
	Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
	Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
	Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));
}

void captureazaCadru()
{
	Camera.grab();
	Camera.retrieve(cadru);
	cvtColor(cadru, cadru, COLOR_BGR2RGB);
}

void schimbaPerspectiva()
{
	#if DEBUG
	line(cadru,sursa[0], sursa[1], Scalar(0,0,255), 2);
	line(cadru,sursa[1], sursa[3], Scalar(0,0,255), 2);
	line(cadru,sursa[3], sursa[2], Scalar(0,0,255), 2);
	line(cadru,sursa[2], sursa[0], Scalar(0,0,255), 2);
	#endif
	
	matriceTransformarePerpectiva = getPerspectiveTransform(sursa, destinatie);
	warpPerspective(cadru, cadruVedereDeDeasupra, matriceTransformarePerpectiva, Size(400,240));
}

void obtineCadruFinal()
{
	int valPragThresh = 165;//noaptea  = 100,ziua =150, a mers cu 166
	int valPragCanny = 130;//noaptea  = 150, 250<ziua  <= 258, seara =200
	cvtColor(cadruVedereDeDeasupra, cadruGri, COLOR_RGB2GRAY);
	inRange(cadruGri, valPragThresh, valPragThresh+20, cadruPrag);
	Canny(cadruGri,cadruMargine, valPragCanny, 2*valPragCanny, 3, false);
	add(cadruPrag, cadruMargine, cadruFinal);
	cvtColor(cadruFinal, cadruFinal, COLOR_GRAY2RGB);
	cvtColor(cadruFinal, cadruFinalCopie, COLOR_RGB2BGR);   //used in creeazaHistograma function only
	cvtColor(cadruFinal, cadruFinalCopie1, COLOR_RGB2BGR);   //used in creeazaHistograma function only
	
}

void creeazaHistograma()
{
	histogramaBanda.resize(400);
	histogramaBanda.clear();
	
	for(int i=0; i<400; i++)       //frame.size().width = 400
	{
		ROIBanda = cadruFinalCopie(Rect(i,140,1,100));
		divide(255, ROIBanda, ROIBanda);
		histogramaBanda.push_back((int)(sum(ROIBanda)[0])); 
	}
	
	histogramaFinalBanda.resize(400);
        histogramaFinalBanda.clear();
	for (int i = 0; i < 400; i++)       
	{
		ROIFinalBanda = cadruFinalCopie1(Rect(i, 0, 1, 240));   
		divide(255, ROIFinalBanda, ROIFinalBanda);       
		histogramaFinalBanda.push_back((int)(sum(ROIFinalBanda)[0]));  
		
	
	}
	   finalBanda = sum(histogramaFinalBanda)[0];
	   //cout<<"Lane END = "<<finalBanda<<'\n';
}

void cautaBanda()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histogramaBanda.begin(), histogramaBanda.begin() + frameCenter -4);
    pozBandaStg = distance(histogramaBanda.begin(), LeftPtr); 
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histogramaBanda.begin() +frameCenter+4, histogramaBanda.end());
    pozBandaDrpt = distance(histogramaBanda.begin(), RightPtr);
    
    #if DEBUG
    line(cadruFinal, Point2f(pozBandaStg, 0), Point2f(pozBandaStg, 240), Scalar(0, 255,0), 2);
    line(cadruFinal, Point2f(pozBandaDrpt, 0), Point2f(pozBandaDrpt, 240), Scalar(0,255,0), 2); 
    #endif
}

void calculeazaCentruBanda()
{
    centruBanda = (pozBandaDrpt-pozBandaStg)/2 +pozBandaStg;
    
    #if DEBUG
    line(cadruFinal, Point2f(centruBanda,0), Point2f(centruBanda,240), Scalar(0,255,0), 3);
    line(cadruFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);
    #endif

    Rezultat = centruBanda-frameCenter;
}


/* aceasta functie face ca valoarea rezultatatului sa nu mai varieze haotic de la 3 la -61 sau 
 * alte lucruri de felul acesta imposibile generate de faptul ca-si pierde o margine a drumului.
*/
void filtreaza() {
    static int rezultat02 = 0;
    static  const int variatie = 15;
    if (Rezultat < (rezultat02-variatie) || Rezultat > (rezultat02+variatie))
        Rezultat = rezultat02;
    rezultat02 = Rezultat;
}


int main(int argc, char **argv) {
	int file_i2c;
	int length;
	unsigned char buffer[1] = {0};
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return -1;
	}
	
	int addr = 0x08;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return -1;
	}

	
	//----- WRITE BYTES -----
	seteazaParamCamera(argc,argv, Camera);
	cout << "Conectare la camera" << '\n';
    if (!Camera.open())
    {
        cout << "Esec la conectare" << '\n';
    }
	cout << "ID camera = " << Camera.getId() << '\n';
    
    int sterge = 0;
    while (1)
    {
	auto inceputT = std::chrono::system_clock::now();

	//prelucrare imagine
        captureazaCadru();
        schimbaPerspectiva();
        obtineCadruFinal();
        creeazaHistograma();
        cautaBanda();
        calculeazaCentruBanda();
        filtreaza();
        //terminare prelucrare imagine
        
        //Rezultat < 0 - masina trebuie sa faca stanga
	//Rezultat > 0 - masina trebuie sa faca dreapta
	//trimitere comanda si prelucrare rezultat
        int maxError = 30;
        //limitarea comenzii
        Rezultat = Rezultat < (-1*maxError) ? (-1*maxError) : Rezultat;
        Rezultat = Rezultat > maxError ? maxError : Rezultat;
        
        int comanda = Rezultat;
        // trecere in [0,2*maxError] daca este comanda stanga, dreapta sau inainte si 2*maxError+1 = stop;
        comanda += maxError;
	//-----sterge
	comanda = maxError;
	//-----stop sterge
	buffer[0] = (char)comanda;
	length = 1;			//<<< Number of bytes to write
	if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
		perror("Eroarea este : ");
	}
	
	
	//afisare vizuala imagini de la camera prelucrate sau nu
	// este bine ca aceasta parte sa fie comentata la rulare deoarece ajuta la marirea vitezei programului
	//si folosita doar la verificarea rularii corecte a programului
	ss.str(" ");
        ss.clear();
        ss << "rezultat = " << Rezultat;
        putText(cadru, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);
	
	#if DEBUG
        namedWindow("Original", WINDOW_KEEPRATIO);
        moveWindow("Original", 0, 100);
        resizeWindow("Original", 640, 480);
        imshow("Original", cadru);

        namedWindow("Vedere de sus", WINDOW_KEEPRATIO);
        moveWindow("Vedere de sus", 640, 100);
        resizeWindow("Vedere de sus", 640, 480);
        imshow("Vedere de sus", cadruMargine);

        namedWindow("Final", WINDOW_KEEPRATIO);
        moveWindow("Final", 1280, 100);
        resizeWindow("Final", 640, 480);
        imshow("Final", cadruFinal);
	
	waitKey(1);
	#endif
	
        auto finalT = std::chrono::system_clock::now();
        std::chrono::duration<double> durata_secunde = finalT - inceputT;

        float t = durata_secunde.count();
        int FPS = 1 / t;
        cout << "FPS = " << FPS << endl;
	}
	close(file_i2c);
	return 0;
}
