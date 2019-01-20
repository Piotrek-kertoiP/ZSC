#include "Arduino.h"
#include <Wire.h>
#include <Accelerometer.h>

Accelerometer::Accelerometer()
{
  I2Cinterface = &Wire;
}

bool Accelerometer::begin() {

  I2Cinterface->begin();

  /* Check connection */
  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
  if (deviceid != 0x33){return false;}
  
  // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL1, 0x07);
  // 400Hz rate
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
  return true;
}
void Accelerometer::read(void) {			//zmienione, Piotrek
	// read x y z at once

	I2Cinterface->beginTransmission(LIS3DH_DEFAULT_ADDRESS);	//Funkcja rozpoczynająca transmisję danych do danego urządzenia podrzędnego. 
																//Jako argument należy podać adres urządzenia podrzędnego. 
	
	//LIS3DH_REG_OUT_X_L = 0x28 = 40 = 101000 (binarnie) = 00101000 (binarnie)
	//0x80 for autoincrement //0x80 = 128 = [10000000]binarnie// | to "bitwise OR"
	// 00101000 | 10000000 = 10101000 (binarnie) = 168 = 0xa8
    //I2Cinterface->write(LIS3DH_REG_OUT_X_L | 0x80); //tak było
	I2Cinterface->write(0xa8);				//Funkcja która wysyła dane przez I2C.
											//Argument to wartość, którą chcemy wysłać
    
	I2Cinterface->endTransmission();		//Funkcja kończąca wysyłanie danych do urządzenia podrzędnego.

    I2Cinterface->requestFrom(0x18, 6);		//Funkcja która wysyła żądanie wysłania danych przez urządzenie podrzędne. 
											//Argumenty :
											//Adres 
											//Ilość bajtów które chcemy odebrać 
											
    x = I2Cinterface->read(); 				//Funkcja która zwraca pierwszy bajt z bufora odebranych danych
	x = x | (((uint16_t)I2Cinterface->read()) << 8);// x:= x[bin] x[bin] 
													//przesunięcie bitowe o 8 miejsc (tak jak mnożenie *2^8) + dodanie w miejscu 8 najmłodszych bitów wart. x
    y = I2Cinterface->read(); 
	y = y | (((uint16_t)I2Cinterface->read()) << 8);
    z = I2Cinterface->read(); 
	z = z | (((uint16_t)I2Cinterface->read()) << 8);
  
	//x * (2^8 + 1) / 8190
	uint8_t range = LIS3DH_RANGE_4_G ;
	uint16_t divider = 8190;				//8190 = 1111111111110 (binarnie)
											//8190 (dzielnik) ma 13 bitów, a dzielna (x,y,z) ma 16 bitów - zatem wynik będzie miał 3 bity + reszta z dzielenia;
											//liczby 3-bitowe to zakres 8 wartości, a więc +/-4G (+ część po przecinku)
	x_g = (float)x / divider;
	y_g = (float)y / divider;
	z_g = (float)z / divider;
}

void Accelerometer::setRange(uint8_t range)		//dodane, Piotrek
{
  //w rejestrze LIS3DH_REG_CTRL4 jest zapisana wartość przedziału odczytywanych wartości przyspieszenia w postaci (stara wartość (4 bity), obecna wartość (4 bity) )
  //gdy ustalamy nową wartość, to obecna wartość staje się starą, a na jej miejsce wpisujemy "nową nową"
  //dlatego najpierw robimy obcięcie 4 najstarszych bitów i przesunięcie bitowe o 4 miejsca w lewo a potem w miejsce 4 najmłodszych bitów wpisujemy nową wartość
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r = r & ~(0x30);								//& to bitowe AND	//~ to bitowe NOT
  //0x30 = 48 = 110000 (binarnie); ~(0b110000) = 001111 (binarnie) = 00001111 (binarnie)
  //r & 00001111 -> obcięcie czterech najstarszych bitów/pozostawienie 4 najmłodszych bitów (to samo co r%16)
  r = r | range << 4;			//bitowe OR (katenacja liczb binarnych, bo r ma 4 bity (abcd), a range << 4 jest postaci efgh0000 i powstaje abcdefgh
  writeRegister8(LIS3DH_REG_CTRL4, r);
}

//-------    Writes 8-bits to the specified destination register   ---------

void Accelerometer::writeRegister8(uint8_t reg, uint8_t value) {
    I2Cinterface->beginTransmission(0x18);
    I2Cinterface->write((uint8_t)reg);
    I2Cinterface->write((uint8_t)value);
    I2Cinterface->endTransmission();
}

//-----  Reads 8-bits from the specified register -----------------

uint8_t Accelerometer::readRegister8(uint8_t reg) {
    uint8_t value;
    I2Cinterface->beginTransmission(0x18);
    I2Cinterface->write((uint8_t)reg);
    I2Cinterface->endTransmission();
    I2Cinterface->requestFrom(0x18, 1);
    value = I2Cinterface->read();   
  return value;
}
