#include "libka.h"

using namespace std;

int BME280::i2cOpen(const string& i2cdevice)
{
    int ret;
    this->file_descriptor = open(i2cdevice.c_str(),O_RDWR);
    if(file_descriptor<0)
    {
        perror("i2c descriptor");
        exit(1);
    }
    if (ret = ioctl(file_descriptor, I2C_SLAVE, this->i2c_addr) < 0)
    {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        
            perror("I2C connection");
            exit(1);
        
    }
    //funkcja zajmujaca sie odczytaniem rejestrow kalibracyjnych
    registerRead();  
    cout<<"Registes loaded"<<endl;
}
/*Z funkcji writeRegister korzystamy zarowno przy czytaniu jak i zapisywaniu informcji z rejestrow.
 * Zgodnie z dokumentacja czujnika przy odczytywaniu danych z rejestrow najpierw musimy wyslac do czujnika adres 
 * samego czujnika jak i adres rejestru z ktorym chcemy sie skomunikowac z bitem R/W jako 1, nastepnie wyslac ponownie 
 * te same informacje z bitem R/W 0. Dlatego sa tu instrukcje if ktore rozrozniaja po stanie zmiennej "transmision",ktory scenariusz ma sie 
 * uruchomic, dodatkowo write register jest dwu argumentowa funkcja ale data jest domyslnie ustawiona na 129 bo nie moze byc na zadna inna wartosc bo nawet wartosc zero
 * uniemozliwialoby korzystanie z domyslnej wartosci bo zero czasem tez sie wysyla do rejestrow.
 */
 
	//Temperature pokazuje mi 22.43
	//wilgotnosc 68.4211
	//a cisnienie 68 tysiecy hehehe
	//problem polega na tym ze te wartosci sie nie zmieniaja mimo odswiezania i ustawienia probkowania rejestrow temp, hum i press
uint8_t BME280::writeRegister(uint8_t reg,uint8_t data)
{
    int ret;
    if(this-> transmision)
    {
        uint8_t buf[3];
        buf[0] = 0x01; // bez bitu staru odczytuje 0 na wszystkich rejestrach.Moze jedna nie jest generowany sprzetowo
        buf[1] =  0x77|0x01; //maskowanie aby LSB zawsze byl 1
        buf[2] = reg;
        
        if(ret = write(file_descriptor,buf,sizeof(buf) ) != sizeof(buf) )
        {
            perror("Blad file descriptor");
            exit(1);
        }
        if (read(file_descriptor, buf, 1) != 1)
        {
            perror("read()");
            exit(1);
        }
        return buf[0]; // zwracanie odczytanych danych z nadpisanego bajtu danych bufora.
    }
    else
    {
        if(data<129) //jaka kolwiek wartosc podana jako drugi argument funkcji bedzie na pewno mniejszy niz 129
        {
            uint8_t buf[4];
            buf[0] = 0x01;
            buf[1] = (0x77>>1)<<1;
            buf[2] = reg;
            buf[3] = data;
           //  cout<<"wielkosc"<<sizeof(buf)<<endl;
          
            if(ret = write(file_descriptor,buf,sizeof(buf) ) != sizeof(buf) )
        {
            perror("Blad send");
            exit(1);
        }
        }
        else
        {
            uint8_t buf[3];
            buf[0] = 0x01;
            buf[1] = (0x77>>1)<<1; //ustawianie ostatniego bitu jako 0/ nie wiem czy to dobrze ale tak wymyslilem
            buf[2] = reg;
            
           
            if(ret = write(file_descriptor,buf,sizeof(buf) ) != sizeof(buf) )
        {
            perror("Blad send");
            exit(1);
        }
        }
    }
    return 1;
}
uint8_t BME280::readRegister(uint8_t reg)
{

    this-> transmision =0; //pierwszy krok, wyslanie do rejestru danych z bitem R/W 0 czyli zapis.
    writeRegister(reg); // wykorzystanie domyslnego argumentu metody.
    this-> transmision=1; //kolejny krok tym razem scenariusz z bitem R/W jako 1
    uint8_t bufor;
    bufor = writeRegister(reg); 
    return bufor;
}
void BME280::registerRead()
{
	calibration.dig_T1 = (((uint16_t)(readRegister(BME280_DIG_T1_MSB_REG)) )<< 8) + readRegister(BME280_DIG_T1_LSB_REG);
    calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
    calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));
    calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
    calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
    calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
    calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
    calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
    calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
    calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
    calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
    calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));
    calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
    calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
    calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
    calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
    calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
    calibration.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));
	cout<<calibration.dig_T1<<endl;
	cout<<calibration.dig_T2<<endl;
	cout<<calibration.dig_T3<<endl;
	cout<<calibration.dig_P1<<endl;
	cout<<calibration.dig_P2<<endl;
	cout<<calibration.dig_P3<<endl;
	cout<<calibration.dig_P4<<endl;
	cout<<calibration.dig_P5<<endl;
	cout<<calibration.dig_P6<<endl;
	cout<<calibration.dig_P7<<endl;
	cout<<calibration.dig_P8<<endl;
	cout<<calibration.dig_P9<<endl;
	cout<<calibration.dig_H1<<endl;
	cout<<calibration.dig_H2<<endl;
	cout<<calibration.dig_H3<<endl;
	cout<<calibration.dig_H4<<endl;
	cout<<calibration.dig_H5<<endl;
	cout<<calibration.dig_H6<<endl;
	char num;
	cin>>num;
}
 
BME280::BME280()
{
	
    i2cOpen("/dev/i2c-1");
}
BME280::~BME280()
{
  
}
float BME280::readTemp( void )
{
    // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
    // t_fine carries fine temperature as global value

    //get the reading (adc_T);
    int32_t adc_T = ((uint32_t)readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

    //By datasheet, calibrate
    int64_t var1, var2;

    var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
            ((int32_t)calibration.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float output = (t_fine * 5 + 128) >> 8;

    output = output / 100;

    return output;
}
float BME280::readHum( void )
{
    // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46. 333 %RH
    int32_t adc_H = ((uint32_t)readRegister(BME280_HUMIDITY_MSB_REG) << 8) | ((uint32_t)readRegister(BME280_HUMIDITY_LSB_REG));

    int32_t var1;
    var1 = (t_fine - ((int32_t)76800));
    var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
              ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                           ((int32_t)calibration.dig_H2) + 8192) >> 14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);

    return (float)(var1>>12) / 1024.0;
}
float BME280::readPress( void )
{
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    int32_t adc_P = ((uint32_t)readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);

    int64_t var1, var2, p_acc;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
    var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p_acc = 1048576 - adc_P;
    p_acc = (((p_acc<<31) - var2)*3125)/var1;
    var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
    var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);

    return (float)p_acc / 256.0;
}
