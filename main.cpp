#include <iostream>
#include "libka.h"
using namespace std;

int main()
{
    BME280 czujnik;
    int8_t sleep         = 0x74;
    int8_t dataHum       = 0x72;
    int8_t dataTemp      = 0x74;
    int8_t dataConfig    = 0x75;
    int8_t id    		  = 0xD0;
	
    czujnik.transmision   = 0; 
    czujnik.writeRegister(sleep,0x00); // zgodnie z dokumentacja usypiamy urzadzenie aby je zaprogramowac.
									   //A nawet gdy to zakomentuje wszystko jest tak samo. Wychodzi na to, ze nic nie programuje
    czujnik.writeRegister(dataHum,0x02);
    czujnik.writeRegister(dataConfig,0x00);
    czujnik.writeRegister(dataTemp,0xFF); // zmiana mode na normal + inne parametry.

    

    while(1)
    {
		usleep(50000);
		system("clear");
		    cout<<"id"<<czujnik.readRegister(id)<<endl;
        cout<<"Temperatura: "<<czujnik.readTemp()<<endl;
        cout<<"Wilgotnosc: "<<czujnik.readHum()<<endl;
        cout<<"Cisnienie: "<<static_cast<int>(czujnik.readPress())<<endl;
    }
    return 0;
}
